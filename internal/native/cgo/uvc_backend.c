#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "ctrl.h"   // video_report_format, video_send_frame
#include "log.h"    // log_info, log_warn, log_error, log_trace
#include "uvc_backend.h" // function prototypes for start/stop/reinit
#include "video_encoder.h" // 统一编码器接口

#include <turbojpeg.h>
#include <libyuv.h>

#ifdef JETKVM_HAVE_MPP
#include "dec_mppjpeg.h"
#endif

// Simple V4L2 MMAP buffer
struct uvc_buf { void *start; size_t length; };

static char uvc_dev_path[256] = "/dev/video1";
static int uvc_width = 1280;
static int uvc_height = 720;
static int uvc_fps = 15;
static int uvc_fd = -1;
static struct uvc_buf *uvc_buffers = NULL;
static int uvc_nbufs = 0;
static pthread_t uvc_thread;
static int uvc_running = 0;
static int uvc_thread_started = 0;
static pthread_mutex_t uvc_mutex = PTHREAD_MUTEX_INITIALIZER;
static int uvc_initialized = 0;

// JPEG decoder
static tjhandle tj = NULL;
static unsigned char *yuv_buf = NULL; // I420/NV12 contiguous buffer
static size_t yuv_buf_size = 0;
// MJPEG source YUV buffer as produced by TurboJPEG (sampling may be 420/422/444)
static unsigned char *yuv_src = NULL;
static size_t yuv_src_size = 0;
static int mjpeg_subsamp = -1; // TJSAMP_*
static int mjpeg_colorspace = -1;
static int tj_fail_count = 0;
static int tj_fail_threshold = 20;
static int uvc_warmup_frames_default = 8;
static int uvc_warmup_frames = 0;

// 统一编码器接口
static const encoder_ops_t *encoder = NULL;
static void *encoder_ctx = NULL;
static encoder_type_t encoder_type = ENCODER_TYPE_X264;
static pixel_format_t encoder_input_fmt = PIXEL_FORMAT_I420;

// 编码器配置
static int repeat_headers = 1;
static int bitrate_kbps = 2000;
static int keyint = 60;
static char x264_preset[32] = "ultrafast";
static char x264_tune[32] = "zerolatency";
static char x264_profile[32] = "baseline";

// 解码器选择
static int use_mpp_decoder = 0; // 1 -> use MPP JPEG decoder (dec_mppjpeg), 0 -> TurboJPEG

// input pixel format selection
static uint32_t uvc_pixfmt = V4L2_PIX_FMT_MJPEG; // default MJPG


static int xioctl(int fd, int request, void *arg) {
    int r;
    do { r = ioctl(fd, request, arg); } while (r == -1 && errno == EINTR);
    return r;
}


static void warn_tj_once_per_sec() {
    static time_t last = 0;
    static int suppressed = 0;
    time_t now = time(NULL);
    if (now == last) { suppressed++; return; }
    if (suppressed > 0) {
        log_warn("UVC: tjDecompressToYUV2 failed (suppressed %d)", suppressed);
    } else {
        log_warn("UVC: tjDecompressToYUV2 failed");
    }
    last = now; suppressed = 0;
}

static void warn_uvc_once_per_sec(const char *msg) {
    static time_t last = 0;
    static int suppressed = 0;
    time_t now = time(NULL);
    if (now == last) { suppressed++; return; }
    if (suppressed > 0) {
        log_warn("UVC: %s (suppressed %d)", msg, suppressed);
    } else {
        log_warn("UVC: %s", msg);
    }
    last = now; suppressed = 0;
}

static int find_jpeg_start(const unsigned char *buf, size_t size) {
    if (!buf || size < 2) {
        return -1;
    }

    // Check for a UVC payload header length and a SOI marker right after it.
    unsigned int hdr_len = buf[0];
    if (hdr_len >= 2 && hdr_len <= 64 && hdr_len + 1 < size) {
        if (buf[hdr_len] == 0xFF && buf[hdr_len + 1] == 0xD8) {
            return (int)hdr_len;
        }
    }

    // Fallback: scan the first 512 bytes for JPEG SOI.
    size_t limit = size < 512 ? size : 512;
    for (size_t i = 0; i + 1 < limit; i++) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xD8) {
            return (int)i;
        }
    }

    return -1;
}

static int has_jpeg_eoi(const unsigned char *buf, size_t size) {
    if (!buf || size < 2) {
        return 0;
    }
    size_t start = size > 512 ? size - 512 : 0;
    for (size_t i = size - 2;; i--) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xD9) {
            return 1;
        }
        if (i == start) {
            break;
        }
    }
    return 0;
}

static int init_turbojpeg_handle() {
    if (!tj) {
        tj = tjInitDecompress();
        if (!tj) {
            log_error("UVC: tjInitDecompress failed");
            return -1;
        }
    }
    return 0;
}

static void reset_turbojpeg_state(const char *reason) {
    log_warn("UVC: resetting TurboJPEG decoder (%s)", reason ? reason : "unknown");
    if (tj) {
        tjDestroy(tj);
        tj = NULL;
    }
    if (yuv_src) { free(yuv_src); yuv_src = NULL; }
    yuv_src_size = 0;
    mjpeg_subsamp = -1;
    mjpeg_colorspace = -1;
    tj_fail_count = 0;
    uvc_warmup_frames = 0;
    tj_fail_count = 0;
    (void)init_turbojpeg_handle();
}


static int uvc_open_device() {
    uvc_fd = open(uvc_dev_path, O_RDWR | O_NONBLOCK, 0);
    if (uvc_fd < 0) {
        log_error("UVC: failed to open %s: %s", uvc_dev_path, strerror(errno));
        return -1;
    }
    log_info("UVC: opened %s", uvc_dev_path);
    return 0;
}

static int uvc_set_format() {
    struct v4l2_format fmt; memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = uvc_width;
    fmt.fmt.pix.height = uvc_height;
    fmt.fmt.pix.pixelformat = uvc_pixfmt;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    
    uint32_t requested_fmt = uvc_pixfmt;
    
    if (xioctl(uvc_fd, VIDIOC_S_FMT, &fmt) < 0) {
        log_error("UVC: VIDIOC_S_FMT failed: %s", strerror(errno));
        return -1;
    }
    
    // CRITICAL: Verify the actual format returned by the driver
    if (fmt.fmt.pix.pixelformat != requested_fmt) {
        char req_fourcc[5] = {0}, got_fourcc[5] = {0};
        memcpy(req_fourcc, &requested_fmt, 4);
        memcpy(got_fourcc, &fmt.fmt.pix.pixelformat, 4);
        log_warn("UVC: Requested format %s (0x%08X) but device returned %s (0x%08X)", 
                 req_fourcc, requested_fmt, got_fourcc, fmt.fmt.pix.pixelformat);
        log_warn("UVC: Device does not support requested format, adapting...");
        
        // Update our format to match what device actually provides
        uvc_pixfmt = fmt.fmt.pix.pixelformat;
    }
    
    // Also verify dimensions
    if (fmt.fmt.pix.width != (uint32_t)uvc_width || fmt.fmt.pix.height != (uint32_t)uvc_height) {
        log_warn("UVC: Requested %dx%d but device returned %dx%d, adapting...",
                 uvc_width, uvc_height, fmt.fmt.pix.width, fmt.fmt.pix.height);
        uvc_width = fmt.fmt.pix.width;
        uvc_height = fmt.fmt.pix.height;
    }
    
    log_info("UVC: Format set to %dx%d, pixelformat=0x%08X (%c%c%c%c)",
             fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.pixelformat,
             (fmt.fmt.pix.pixelformat >> 0) & 0xFF,
             (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
             (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
             (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
    
    // Try set FPS
    struct v4l2_streamparm parm; memset(&parm, 0, sizeof parm);
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = uvc_fps;
    xioctl(uvc_fd, VIDIOC_S_PARM, &parm); // best effort
    return 0;
}

static int uvc_init_mmap() {
    struct v4l2_requestbuffers req; memset(&req, 0, sizeof req);
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(uvc_fd, VIDIOC_REQBUFS, &req) < 0) {
        log_error("UVC: VIDIOC_REQBUFS failed: %s", strerror(errno));
        return -1;
    }
    if (req.count < 2) {
        log_error("UVC: insufficient buffer memory");
        return -1;
    }
    uvc_buffers = calloc(req.count, sizeof *uvc_buffers);
    if (!uvc_buffers) return -1;
    for (uvc_nbufs = 0; uvc_nbufs < (int)req.count; uvc_nbufs++) {
        struct v4l2_buffer buf; memset(&buf, 0, sizeof buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = uvc_nbufs;
        if (xioctl(uvc_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            log_error("UVC: VIDIOC_QUERYBUF failed: %s", strerror(errno));
            return -1;
        }
        uvc_buffers[uvc_nbufs].length = buf.length;
        uvc_buffers[uvc_nbufs].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, uvc_fd, buf.m.offset);
        if (uvc_buffers[uvc_nbufs].start == MAP_FAILED) {
            log_error("UVC: mmap failed: %s", strerror(errno));
            return -1;
        }
    }
    // queue all
    for (int i = 0; i < uvc_nbufs; i++) {
        struct v4l2_buffer buf; memset(&buf, 0, sizeof buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(uvc_fd, VIDIOC_QBUF, &buf) < 0) {
            log_error("UVC: VIDIOC_QBUF failed: %s", strerror(errno));
            return -1;
        }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(uvc_fd, VIDIOC_STREAMON, &type) < 0) {
        log_error("UVC: VIDIOC_STREAMON failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

static int init_turbojpeg() {
    if (init_turbojpeg_handle() < 0) {
        return -1;
    }
    // allocate yuv buffer for I420 with minimal padding (1)
    if (!yuv_buf) {
        yuv_buf_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
        yuv_buf = (unsigned char*)malloc(yuv_buf_size);
        if (!yuv_buf) return -1;
    }
    return 0;
}

static void yuyv_to_i420(const unsigned char *src, int width, int height,
                         unsigned char *dst_y, unsigned char *dst_u, unsigned char *dst_v,
                         int stride_y, int stride_uv) {
    YUY2ToI420(src, width * 2,
               dst_y, stride_y,
               dst_u, stride_uv,
               dst_v, stride_uv,
               width, height);
}

static void nv12_to_i420(const unsigned char *src_nv12, int width, int height,
                         unsigned char *dst_y, unsigned char *dst_u, unsigned char *dst_v,
                         int stride_y, int stride_uv) {
    NV12ToI420(src_nv12, width,
               src_nv12 + width * height, width,
               dst_y, stride_y,
               dst_u, stride_uv,
               dst_v, stride_uv,
               width, height);
}

static void i420_to_nv12(const unsigned char *Y, const unsigned char *U, const unsigned char *V,
                         int width, int height, unsigned char *dst_nv12) {
    size_t ysize = (size_t)width * height;
    memcpy(dst_nv12, Y, ysize);
    unsigned char *dst_uv = dst_nv12 + ysize;
    const int cw = width / 2, ch = height / 2;
    for (int y = 0; y < ch; ++y) {
        for (int x = 0; x < cw; ++x) {
            dst_uv[y*width + 2*x + 0] = U[y*cw + x];
            dst_uv[y*width + 2*x + 1] = V[y*cw + x];
        }
    }
}

static int init_encoder() {
    // 获取编码器ops
    encoder = encoder_get_ops(encoder_type);
    if (!encoder) {
        log_error("UVC: failed to get encoder ops for type %d", encoder_type);
        return -1;
    }
    
    // 准备编码器配置
    encoder_config_t config = {0};
    config.width = uvc_width;
    config.height = uvc_height;
    config.fps = uvc_fps;
    config.bitrate_kbps = bitrate_kbps;
    config.keyint = keyint;
    config.repeat_headers = repeat_headers;

    // 根据 UVC 格式和编码器类型选择最佳输入格式
    if (encoder_type == ENCODER_TYPE_MPP) {
        // MPP 编码器：优先使用 YUYV/NV12 直通
        if (uvc_pixfmt == V4L2_PIX_FMT_YUYV) {
            config.input_format = PIXEL_FORMAT_YUYV;  // YUYV 直通！
        } else if (uvc_pixfmt == V4L2_PIX_FMT_NV12) {
            config.input_format = PIXEL_FORMAT_NV12;  // NV12 直通
        } else {
            config.input_format = PIXEL_FORMAT_NV12;  // 其他格式转 NV12
        }
    } else {
        // x264 编码器：仅支持 I420
        config.input_format = PIXEL_FORMAT_I420;
    }

    strncpy(config.x264_preset, x264_preset, sizeof(config.x264_preset) - 1);
    strncpy(config.x264_tune, x264_tune, sizeof(config.x264_tune) - 1);
    strncpy(config.x264_profile, x264_profile, sizeof(config.x264_profile) - 1);
    
    // 初始化编码器
    if (encoder->init(&config, &encoder_ctx) < 0) {
        log_error("UVC: encoder init failed");
        return -1;
    }
    
    // 获取编码器期望的输入格式
    encoder_input_fmt = encoder->get_input_format(encoder_ctx);
    
    const char *fmt_str = "UNKNOWN";
    if (encoder_input_fmt == PIXEL_FORMAT_I420) fmt_str = "I420";
    else if (encoder_input_fmt == PIXEL_FORMAT_NV12) fmt_str = "NV12";
    else if (encoder_input_fmt == PIXEL_FORMAT_YUYV) fmt_str = "YUYV";
    
    log_info("UVC: encoder '%s' initialized, input format=%s", encoder->name, fmt_str);
    
    return 0;
}

static void* uvc_thread_main(void* arg) {
    (void)arg;
    log_info("UVC: streaming thread started");
    // publish video state once
    video_report_format(true, NULL, (u_int16_t)uvc_width, (u_int16_t)uvc_height, (double)uvc_fps);
    while (uvc_running) {
        fd_set fds; FD_ZERO(&fds); FD_SET(uvc_fd, &fds);
        struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
        int r = select(uvc_fd + 1, &fds, NULL, NULL, &tv);
        if (r == -1) {
            if (errno == EINTR) continue;
            log_error("UVC: select failed: %s", strerror(errno));
            pthread_mutex_lock(&uvc_mutex);
            uvc_running = 0;
            pthread_mutex_unlock(&uvc_mutex);
            break;
        } else if (r == 0) {
            continue; // timeout
        }

        struct v4l2_buffer buf; memset(&buf, 0, sizeof buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(uvc_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            log_error("UVC: VIDIOC_DQBUF failed: %s", strerror(errno));
            pthread_mutex_lock(&uvc_mutex);
            uvc_running = 0;
            pthread_mutex_unlock(&uvc_mutex);
            break;
        }
        if (!uvc_buffers || buf.index >= (uint32_t)uvc_nbufs) {
            log_error("UVC: invalid buffer index %u (nbufs=%d)", buf.index, uvc_nbufs);
            pthread_mutex_lock(&uvc_mutex);
            uvc_running = 0;
            pthread_mutex_unlock(&uvc_mutex);
            break;
        }
        if (!uvc_buffers[buf.index].start) {
            log_error("UVC: buffer %u has NULL start pointer", buf.index);
            pthread_mutex_lock(&uvc_mutex);
            uvc_running = 0;
            pthread_mutex_unlock(&uvc_mutex);
            break;
        }
        if (!encoder || !encoder_ctx) {
            log_error("UVC: encoder not initialized");
            pthread_mutex_lock(&uvc_mutex);
            uvc_running = 0;
            pthread_mutex_unlock(&uvc_mutex);
            break;
        }
        if (buf.flags & V4L2_BUF_FLAG_ERROR) {
            warn_uvc_once_per_sec("buffer error flag set, dropping frame");
            goto requeue;
        }
        if (buf.bytesused == 0) {
            warn_uvc_once_per_sec("empty frame bytesused=0, dropping");
            goto requeue;
        }
        if (buf.bytesused > uvc_buffers[buf.index].length) {
            warn_uvc_once_per_sec("bytesused exceeds buffer length, dropping");
            goto requeue;
        }
        if (uvc_warmup_frames > 0) {
            if (uvc_warmup_frames == uvc_warmup_frames_default) {
                log_info("UVC: warmup active, dropping %d frames", uvc_warmup_frames_default);
            }
            uvc_warmup_frames--;
            goto requeue;
        }

        if (uvc_pixfmt == V4L2_PIX_FMT_MJPEG) {
#ifdef JETKVM_HAVE_MPP
            if (use_mpp_decoder) {
                unsigned char* mjpg_ptr = (unsigned char*)uvc_buffers[buf.index].start;
                unsigned long mjpg_size = buf.bytesused;

                dec_mppjpeg_frame_t dec_frame = {0};
                int dr = dec_mppjpeg_decode_zero_copy(mjpg_ptr, mjpg_size, &dec_frame);
                if (dr != 0) {
                    warn_uvc_once_per_sec("mppjpeg zero-copy decode failed, dropping");
                    (void)xioctl(uvc_fd, VIDIOC_QBUF, &buf);
                    continue;
                }
                
                int iw = dec_frame.width;
                int ih = dec_frame.height;
                
                if (iw != uvc_width || ih != uvc_height) {
                    log_warn("UVC: MPP JPEG size %dx%d != requested %dx%d, adapting", iw, ih, uvc_width, uvc_height);
                    uvc_width = iw; uvc_height = ih;
                    // 重建编码器
                    if (encoder && encoder->destroy) {
                        encoder->destroy(&encoder_ctx);
                    }
                    if (init_encoder() < 0) { 
                        log_error("UVC: re-init encoder failed"); 
                        break; 
                    }
                    video_report_format(true, NULL, (u_int16_t)uvc_width, (u_int16_t)uvc_height, (double)uvc_fps);
                }
                
                // ⭐ 零拷贝：传递解码器的 stride 信息（关键优化）
                video_frame_t frame = {0};
                if (encoder_input_fmt == PIXEL_FORMAT_I420) {
                    size_t needed = (size_t)iw * (size_t)ih * 3 / 2;
                    if (yuv_buf_size < needed) {
                        unsigned char *nb = (unsigned char*)realloc(yuv_buf, needed);
                        if (!nb) {
                            log_error("UVC: realloc yuv_buf failed for MPP decode");
                            (void)xioctl(uvc_fd, VIDIOC_QBUF, &buf);
                            continue;
                        }
                        yuv_buf = nb;
                        yuv_buf_size = needed;
                    }
                    const unsigned char *src_y = (const unsigned char*)dec_frame.data_ptr;
                    const unsigned char *src_uv = (const unsigned char*)dec_frame.data_ptr +
                        ((size_t)dec_frame.hor_stride * (size_t)dec_frame.ver_stride);
                    unsigned char *dst_y = yuv_buf;
                    unsigned char *dst_u = dst_y + (size_t)iw * (size_t)ih;
                    unsigned char *dst_v = dst_u + ((size_t)iw / 2) * ((size_t)ih / 2);
                    NV12ToI420(src_y, dec_frame.hor_stride,
                               src_uv, dec_frame.hor_stride,
                               dst_y, iw,
                               dst_u, iw / 2,
                               dst_v, iw / 2,
                               iw, ih);
                    frame.data = yuv_buf;
                    frame.size = needed;
                    frame.format = PIXEL_FORMAT_I420;
                    frame.width = iw;
                    frame.height = ih;
                    frame.stride_y = iw;
                    frame.stride_uv = iw / 2;
                    frame.pts_us = 0;
                } else {
                    frame.data = dec_frame.data_ptr;
                    frame.size = (size_t)dec_frame.hor_stride * dec_frame.ver_stride * 3 / 2;
                    frame.format = PIXEL_FORMAT_NV12;
                    frame.width = iw;
                    frame.height = ih;
                    frame.stride_y = dec_frame.hor_stride;
                    frame.stride_uv = dec_frame.hor_stride;
                    frame.pts_us = 0;
                    // 把 MPP 原生句柄传给编码器，允许零拷贝
                    frame.rk_mpp_frame = dec_frame.mpp_frame;
                    frame.rk_mpp_buffer = dec_frame.mpp_buffer;
                }
                
                encoded_packet_t packet = {0};
                int enc_ret = encoder->encode(encoder_ctx, &frame, &packet);
                if (enc_ret < 0) {
                    log_warn("UVC: encoder encode failed (MPP decoder path)");
                } else if (packet.data && packet.size > 0) {
                    video_send_frame(packet.data, (ssize_t)packet.size);
                    encoder_packet_free(&packet);
                }
                goto requeue;
            }
#endif
            unsigned char* mjpg_ptr = (unsigned char*)uvc_buffers[buf.index].start;
            unsigned long mjpg_size = buf.bytesused;
            if (mjpeg_subsamp >= 0 && (!yuv_src || yuv_src_size == 0)) {
                log_warn("UVC: MJPEG state lost, reinitializing decoder buffers");
                mjpeg_subsamp = -1;
                mjpeg_colorspace = -1;
            }
            if (!tj) {
                warn_uvc_once_per_sec("TurboJPEG handle not initialized, dropping");
                goto requeue;
            }
            if (mjpg_size < 2) {
                warn_uvc_once_per_sec("MJPEG frame too small, dropping");
                goto requeue;
            }
            unsigned int hdr_len = mjpg_ptr[0];
            if (hdr_len >= 2 && hdr_len <= 64 && hdr_len + 1 < mjpg_size) {
                if (mjpg_ptr[1] & 0x40) {
                    warn_uvc_once_per_sec("MJPEG payload error flag set, dropping");
                    goto requeue;
                }
            }
            int jpeg_offset = find_jpeg_start(mjpg_ptr, mjpg_size);
            if (jpeg_offset > 0) {
                log_debug("UVC: skipping %d-byte MJPEG header", jpeg_offset);
                mjpg_ptr += jpeg_offset;
                mjpg_size -= (unsigned long)jpeg_offset;
            } else if (jpeg_offset < 0) {
                log_warn("UVC: MJPEG frame missing SOI marker, dropping");
                (void)xioctl(uvc_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            if (!has_jpeg_eoi(mjpg_ptr, mjpg_size)) {
                warn_uvc_once_per_sec("MJPEG frame missing EOI marker, dropping");
                goto requeue;
            }
            int flags = 0;
            // Parse subsampling on first frame and allocate buffers accordingly
            int iw=uvc_width, ih=uvc_height; // image width/height from header
            if (mjpeg_subsamp < 0) {
                int subs=0, cs=0;
                if (tjDecompressHeader3(tj, mjpg_ptr, mjpg_size, &iw, &ih, &subs, &cs) != 0) {
                    tj_fail_count++;
                    warn_tj_once_per_sec();
                    if (tj_fail_count >= tj_fail_threshold) {
                        reset_turbojpeg_state("header decode failed");
                    }
                    (void)xioctl(uvc_fd, VIDIOC_QBUF, &buf);
                    continue;
                }
                tj_fail_count = 0;
                mjpeg_subsamp = subs; mjpeg_colorspace = cs;
                // If device ignored S_FMT and delivered different size, adapt encoder/buffers
                if (iw != uvc_width || ih != uvc_height) {
                    log_warn("UVC: MJPEG header size %dx%d != requested %dx%d, adapting",
                             iw, ih, uvc_width, uvc_height);
                    uvc_width = iw; uvc_height = ih;
                    if (yuv_buf) { free(yuv_buf); yuv_buf = NULL; }
                    
                    // 重建编码器（统一处理，无论x264还是MPP）
                    if (encoder && encoder->destroy) {
                        encoder->destroy(&encoder_ctx);
                    }
                    if (init_encoder() < 0) { 
                        log_error("UVC: re-init encoder failed"); 
                        break; 
                    }
                    
                    video_report_format(true, NULL, (u_int16_t)uvc_width, (u_int16_t)uvc_height, (double)uvc_fps);
                }
                if (!yuv_buf) {
                    yuv_buf_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
                    yuv_buf = (unsigned char*)malloc(yuv_buf_size);
                    if (!yuv_buf) {
                        log_error("UVC: malloc yuv_buf failed");
                        pthread_mutex_lock(&uvc_mutex);
                        uvc_running = 0;
                        pthread_mutex_unlock(&uvc_mutex);
                        break;
                    }
                }
                // Always allocate yuv_src for TurboJPEG YUV output regardless of subsampling
                // so that tjDecompressToYUV2 always has a valid destination.
                if (yuv_src) { free(yuv_src); yuv_src = NULL; }
                yuv_src_size = tjBufSizeYUV2(iw, 1, ih, mjpeg_subsamp);
                yuv_src = (unsigned char*)malloc(yuv_src_size);
                if (!yuv_src) {
                    log_error("UVC: malloc yuv_src failed");
                    pthread_mutex_lock(&uvc_mutex);
                    uvc_running = 0;
                    pthread_mutex_unlock(&uvc_mutex);
                    break;
                }
                log_info("UVC: MJPEG subsamp=%d colorspace=%d", mjpeg_subsamp, mjpeg_colorspace);
            }
            // Decompress with actual header size (iw/ih); if not known (subsample already set), assume uvc_width/height
            if (mjpeg_subsamp >= 0) { iw = uvc_width; ih = uvc_height; }
            if (tjDecompressToYUV2(tj, mjpg_ptr, mjpg_size, yuv_src, iw, 1, ih, flags) != 0) {
                tj_fail_count++;
                warn_tj_once_per_sec();
                if (tj_fail_count >= tj_fail_threshold) {
                    reset_turbojpeg_state("decode failed");
                }
                (void)xioctl(uvc_fd, VIDIOC_QBUF, &buf);
                continue;
            }
            tj_fail_count = 0;
            // Convert to I420 in yuv_buf
            unsigned char* Yd = yuv_buf;
            unsigned char* Ud = Yd + uvc_width * uvc_height;
            unsigned char* Vd = Ud + (uvc_width/2) * (uvc_height/2);
            unsigned char* Ys = yuv_src;
            memcpy(Yd, Ys, (size_t)uvc_width * uvc_height);
            if (mjpeg_subsamp == TJSAMP_420) {
                int cw = uvc_width/2, ch = uvc_height/2;
                unsigned char* Us = Ys + (uvc_width * uvc_height);
                unsigned char* Vs = Us + cw * ch;
                memcpy(Ud, Us, (size_t)cw * ch);
                memcpy(Vd, Vs, (size_t)cw * ch);
            } else if (mjpeg_subsamp == TJSAMP_422) {
                int cw = uvc_width/2;
                unsigned char* Us = Ys + (uvc_width * uvc_height);
                unsigned char* Vs = Us + cw * uvc_height; // 4:2:2 has full chroma height
                for (int y2 = 0; y2 < uvc_height; y2 += 2) {
                    unsigned char* u0 = Us + y2 * cw;
                    unsigned char* u1 = u0 + cw;
                    unsigned char* v0 = Vs + y2 * cw;
                    unsigned char* v1 = v0 + cw;
                    unsigned char* du = Ud + (y2/2) * cw;
                    unsigned char* dv = Vd + (y2/2) * cw;
                    for (int x2 = 0; x2 < cw; ++x2) {
                        du[x2] = (unsigned char)(((int)u0[x2] + (int)u1[x2] + 1) >> 1);
                        dv[x2] = (unsigned char)(((int)v0[x2] + (int)v1[x2] + 1) >> 1);
                    }
                }
            } else { // TJSAMP_444 and others -> box filter 2x2
                int cw = uvc_width/2;
                unsigned char* Us = Ys + (uvc_width * uvc_height);
                unsigned char* Vs = Us + uvc_width * uvc_height;
                for (int y2 = 0; y2 < uvc_height; y2 += 2) {
                    unsigned char* u0 = Us + y2 * uvc_width;
                    unsigned char* u1 = u0 + uvc_width;
                    unsigned char* v0 = Vs + y2 * uvc_width;
                    unsigned char* v1 = v0 + uvc_width;
                    unsigned char* du = Ud + (y2/2) * cw;
                    unsigned char* dv = Vd + (y2/2) * cw;
                    for (int x2 = 0; x2 < uvc_width; x2 += 2) {
                        int uavg = u0[x2] + u0[x2+1] + u1[x2] + u1[x2+1];
                        int vavg = v0[x2] + v0[x2+1] + v1[x2] + v1[x2+1];
                        du[x2/2] = (unsigned char)((uavg + 2) >> 2);
                        dv[x2/2] = (unsigned char)((vavg + 2) >> 2);
                    }
                }
            }
        } else if (uvc_pixfmt == V4L2_PIX_FMT_YUYV) {
            size_t expected_size = (size_t)uvc_width * (size_t)uvc_height * 2;
            if (buf.bytesused < expected_size || uvc_buffers[buf.index].length < expected_size) {
                log_warn("UVC: short YUYV frame bytesused=%u expected=%zu buf_len=%zu",
                         buf.bytesused, expected_size, uvc_buffers[buf.index].length);
                goto requeue;
            }
            // YUYV → MPP 直通（零转换）
            if (encoder_input_fmt == PIXEL_FORMAT_YUYV) {
                const unsigned char *yuyv = (const unsigned char*)uvc_buffers[buf.index].start;
                
                video_frame_t frame = {0};
                frame.data = (uint8_t*)yuyv;
                frame.size = (size_t)uvc_width * uvc_height * 2;  // YUYV: 2 bytes/pixel
                frame.format = PIXEL_FORMAT_YUYV;
                frame.width = uvc_width;
                frame.height = uvc_height;
                frame.stride_y = uvc_width * 2;  // YUYV stride (bytes)
                frame.stride_uv = 0;             // YUYV 没有独立 UV plane
                frame.pts_us = 0;
                
                encoded_packet_t packet = {0};
                int enc_ret = encoder->encode(encoder_ctx, &frame, &packet);
                if (enc_ret < 0) {
                    log_warn("UVC: YUYV encode failed");
                } else if (packet.data && packet.size > 0) {
                    video_send_frame(packet.data, (ssize_t)packet.size);
                    encoder_packet_free(&packet);
                }
                goto requeue;
            }
            
            // libyuv CPU 转换
            const unsigned char *yuyv = (const unsigned char*)uvc_buffers[buf.index].start;
            if (!yuv_buf) {
                yuv_buf_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
                yuv_buf = (unsigned char*)malloc(yuv_buf_size);
                if (!yuv_buf) {
                    log_error("UVC: malloc yuv_buf failed");
                    pthread_mutex_lock(&uvc_mutex);
                    uvc_running = 0;
                    pthread_mutex_unlock(&uvc_mutex);
                    break;
                }
            }
            unsigned char* Y = yuv_buf;
            unsigned char* U = Y + uvc_width * uvc_height;
            unsigned char* V = U + (uvc_width/2) * (uvc_height/2);
            yuyv_to_i420(yuyv, uvc_width, uvc_height, Y, U, V, uvc_width, uvc_width/2);
        } else if (uvc_pixfmt == V4L2_PIX_FMT_NV12) {
            size_t expected_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
            if (buf.bytesused < expected_size || uvc_buffers[buf.index].length < expected_size) {
                log_warn("UVC: short NV12 frame bytesused=%u expected=%zu buf_len=%zu",
                         buf.bytesused, expected_size, uvc_buffers[buf.index].length);
                goto requeue;
            }
            if (encoder_input_fmt == PIXEL_FORMAT_NV12) {
                const unsigned char *nv12 = (const unsigned char*)uvc_buffers[buf.index].start;
                
                video_frame_t frame = {0};
                frame.data = (uint8_t*)nv12;
                frame.size = (size_t)uvc_width * uvc_height * 3 / 2;
                frame.format = PIXEL_FORMAT_NV12;
                frame.width = uvc_width;
                frame.height = uvc_height;
                frame.stride_y = uvc_width;
                frame.stride_uv = uvc_width;
                frame.pts_us = 0;
                
                encoded_packet_t packet = {0};
                int enc_ret = encoder->encode(encoder_ctx, &frame, &packet);
                if (enc_ret < 0) {
                    log_warn("UVC: encoder encode failed");
                } else if (packet.data && packet.size > 0) {
                    video_send_frame(packet.data, (ssize_t)packet.size);
                    encoder_packet_free(&packet);
                }
                goto requeue;
            }
            
            // x264 需要 I420，执行转换
            const unsigned char *nv12 = (const unsigned char*)uvc_buffers[buf.index].start;
            if (!yuv_buf) {
                yuv_buf_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
                yuv_buf = (unsigned char*)malloc(yuv_buf_size);
                if (!yuv_buf) {
                    log_error("UVC: malloc yuv_buf failed");
                    pthread_mutex_lock(&uvc_mutex);
                    uvc_running = 0;
                    pthread_mutex_unlock(&uvc_mutex);
                    break;
                }
            }
            unsigned char* Y = yuv_buf;
            unsigned char* U = Y + uvc_width * uvc_height;
            unsigned char* V = U + (uvc_width/2) * (uvc_height/2);
            nv12_to_i420(nv12, uvc_width, uvc_height, Y, U, V, uvc_width, uvc_width/2);
        } else {
            log_warn("UVC: unsupported pixel format 0x%x, skipping frame", uvc_pixfmt);
            (void)xioctl(uvc_fd, VIDIOC_QBUF, &buf);
            continue;
        }

        // 统一编码逻辑
        // yuv_buf当前包含I420数据，需要根据编码器要求的格式进行转换
        static unsigned char *encoder_buf = NULL;
        static size_t encoder_buf_size = 0;
        size_t frame_size = (size_t)uvc_width * uvc_height * 3 / 2;
        
        // 确保编码器buffer足够大
        if (encoder_buf_size < frame_size) {
            unsigned char *nb = (unsigned char*)realloc(encoder_buf, frame_size);
            if (!nb) {
                log_error("UVC: alloc encoder_buf failed");
                pthread_mutex_lock(&uvc_mutex);
                uvc_running = 0;
                pthread_mutex_unlock(&uvc_mutex);
                break;
            }
            encoder_buf = nb; encoder_buf_size = frame_size;
        }
        
        // 根据编码器期望的格式准备数据
        if (encoder_input_fmt == PIXEL_FORMAT_NV12) {
            // I420 -> NV12转换
            unsigned char* y = yuv_buf;
            unsigned char* u = y + (uvc_width * uvc_height);
            unsigned char* v = u + (uvc_width/2) * (uvc_height/2);
            i420_to_nv12(y, u, v, uvc_width, uvc_height, encoder_buf);
        } else {
            // I420格式，直接使用
            memcpy(encoder_buf, yuv_buf, frame_size);
        }
        
        // 准备video_frame
        video_frame_t frame = {0};
        frame.data = encoder_buf;
        frame.size = frame_size;
        frame.format = encoder_input_fmt;
        frame.width = uvc_width;
        frame.height = uvc_height;
        frame.stride_y = uvc_width;
        frame.stride_uv = (encoder_input_fmt == PIXEL_FORMAT_NV12) ? uvc_width : (uvc_width / 2);
        frame.pts_us = 0;
        
        // 编码
        encoded_packet_t packet = {0};
        int enc_ret = encoder->encode(encoder_ctx, &frame, &packet);
        if (enc_ret < 0) {
            log_warn("UVC: encoder encode failed");
        } else if (packet.data && packet.size > 0) {
            // 发送编码数据
            video_send_frame(packet.data, (ssize_t)packet.size);
            encoder_packet_free(&packet);
        }

requeue:
        if (xioctl(uvc_fd, VIDIOC_QBUF, &buf) < 0) {
            log_error("UVC: VIDIOC_QBUF failed: %s", strerror(errno));
            pthread_mutex_lock(&uvc_mutex);
            uvc_running = 0;
            pthread_mutex_unlock(&uvc_mutex);
            break;
        }
    }
    pthread_mutex_lock(&uvc_mutex);
    uvc_running = 0;
    pthread_mutex_unlock(&uvc_mutex);
    log_info("UVC: streaming thread exiting");
    return NULL;
}

int uvc_init_from_env() {
    if (uvc_initialized) {
        log_warn("UVC: init called while already initialized");
        return 0;
    }
    mjpeg_subsamp = -1;
    mjpeg_colorspace = -1;

    const char* s;
    s = getenv("JETKVM_UVC_DEVICE"); if (s && s[0]) { strncpy(uvc_dev_path, s, sizeof(uvc_dev_path)-1); uvc_dev_path[sizeof(uvc_dev_path)-1] = '\0'; }
    s = getenv("JETKVM_UVC_WIDTH"); if (s) { int v = atoi(s); if (v>0) uvc_width = v; }
    s = getenv("JETKVM_UVC_HEIGHT"); if (s) { int v = atoi(s); if (v>0) uvc_height = v; }
    s = getenv("JETKVM_UVC_FPS"); if (s) { int v = atoi(s); if (v>0) uvc_fps = v; }
    s = getenv("JETKVM_UVC_FORMAT"); 
    if (s && s[0]) { 
        if (strcasecmp(s, "YUYV")==0 || strcasecmp(s, "YUY2")==0) 
            uvc_pixfmt = V4L2_PIX_FMT_YUYV; 
        else if (strcasecmp(s, "NV12")==0) 
            uvc_pixfmt = V4L2_PIX_FMT_NV12;
        else if (strcasecmp(s, "MJPEG")==0 || strcasecmp(s, "MJPG")==0)
            uvc_pixfmt = V4L2_PIX_FMT_MJPEG;
        else {
            log_warn("UVC: Unknown format '%s', using MJPEG", s);
            uvc_pixfmt = V4L2_PIX_FMT_MJPEG;
        }
    }
    s = getenv("JETKVM_BITRATE_KBPS"); if (s) { int v = atoi(s); if (v>0) bitrate_kbps = v; }
    s = getenv("JETKVM_KEYINT"); if (s) { int v = atoi(s); if (v>0) keyint = v; }
    s = getenv("JETKVM_REPEAT_HEADERS"); if (s) { int v = atoi(s); repeat_headers = (v!=0); }
    s = getenv("JETKVM_X264_PRESET"); if (s && s[0]) { strncpy(x264_preset, s, sizeof(x264_preset)-1); }
    s = getenv("JETKVM_X264_TUNE"); if (s && s[0]) { strncpy(x264_tune, s, sizeof(x264_tune)-1); }
    s = getenv("JETKVM_X264_PROFILE"); if (s && s[0]) { strncpy(x264_profile, s, sizeof(x264_profile)-1); }
    s = getenv("JETKVM_UVC_WARMUP_FRAMES");
    if (s && s[0]) {
        int v = atoi(s);
        if (v >= 0) {
            uvc_warmup_frames_default = v;
        }
    }
    s = getenv("JETKVM_TJ_RESET_FAILS");
    if (s && s[0]) {
        int v = atoi(s);
        if (v > 0) {
            tj_fail_threshold = v;
        }
    }
    // 编码器选择：通过JETKVM_ENCODER环境变量（"x264" 或 "mpp"）
    s = getenv("JETKVM_ENCODER");
    if (s && s[0]) {
        encoder_type = encoder_type_from_name(s);
    } else {
        encoder_type = ENCODER_TYPE_X264;  // 默认x264
    }
    log_info("UVC: Selected encoder type: %s", 
             encoder_type == ENCODER_TYPE_MPP ? "mpp" : "x264");
    
    // 解码器选择：MJPEG的MPP JPEG解码器或TurboJPEG
    // 默认：如果编码器是MPP，则解码器也用MPP
    s = getenv("JETKVM_DECODER");
    if (s && s[0]) {
        if (strcasecmp(s, "mpp") == 0) {
#ifdef JETKVM_HAVE_MPP
            use_mpp_decoder = 1;
#else
            log_warn("UVC: decoder=mpp requested but not enabled at build time; using TurboJPEG");
#endif
        } else if (strcasecmp(s, "turbojpeg") == 0) {
            use_mpp_decoder = 0;
        }
    } else {
        // 默认：编码器是MPP时，解码器也用MPP
        use_mpp_decoder = (encoder_type == ENCODER_TYPE_MPP);
    }

    if (uvc_open_device() < 0) return -1;
    
    // Set format first - this may change uvc_pixfmt if device doesn't support requested format
    if (uvc_set_format() < 0) { uvc_shutdown(); return -1; }
    
    // Now initialize decoders based on ACTUAL format returned by device
    if (uvc_pixfmt == V4L2_PIX_FMT_MJPEG) {
        log_info("UVC: Using MJPEG input format");
#ifdef JETKVM_HAVE_MPP
        if (use_mpp_decoder) {
            log_info("UVC: Using MPP JPEG decoder");
            if (dec_mppjpeg_init(uvc_width, uvc_height) < 0) {
                log_error("UVC: Failed to init MPP JPEG decoder");
                uvc_shutdown();
                return -1;
            }
        } else {
            log_info("UVC: Using TurboJPEG software decoder");
            if (init_turbojpeg() < 0) {
                log_error("UVC: Failed to init TurboJPEG decoder");
                uvc_shutdown();
                return -1;
            }
        }
#else
        log_info("UVC: Using TurboJPEG software decoder");
        if (init_turbojpeg() < 0) {
            log_error("UVC: Failed to init TurboJPEG decoder");
            uvc_shutdown();
            return -1;
        }
#endif
    } else if (uvc_pixfmt == V4L2_PIX_FMT_YUYV) {
        log_info("UVC: Using YUYV input format (will convert to I420)");
        yuv_buf_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
        yuv_buf = (unsigned char*)malloc(yuv_buf_size);
        if (!yuv_buf) {
            log_error("UVC: Failed to allocate YUV conversion buffer");
            uvc_shutdown();
            return -1;
        }
    } else if (uvc_pixfmt == V4L2_PIX_FMT_NV12) {
        log_info("UVC: Using NV12 input format (will convert to I420)");
        yuv_buf_size = (size_t)uvc_width * (size_t)uvc_height * 3 / 2;
        yuv_buf = (unsigned char*)malloc(yuv_buf_size);
        if (!yuv_buf) {
            log_error("UVC: Failed to allocate YUV conversion buffer");
            uvc_shutdown();
            return -1;
        }
    } else {
        log_error("UVC: Unsupported pixel format 0x%08X", uvc_pixfmt);
        uvc_shutdown();
        return -1;
    }
    
    if (uvc_init_mmap() < 0) { uvc_shutdown(); return -1; }
    
    // 统一初始化编码器
    if (init_encoder() < 0) {
        log_error("UVC: init encoder failed");
        uvc_shutdown();
        return -1;
    }
    
    const char *fmt_name = "UNKNOWN";
    if (uvc_pixfmt == V4L2_PIX_FMT_MJPEG) fmt_name = "MJPEG";
    else if (uvc_pixfmt == V4L2_PIX_FMT_YUYV) fmt_name = "YUYV";
    else if (uvc_pixfmt == V4L2_PIX_FMT_NV12) fmt_name = "NV12";

    uvc_initialized = 1;
    uvc_warmup_frames = uvc_warmup_frames_default;
    log_info("UVC: initialized device=%s %dx%d@%dfps fmt=%s encoder=%s bitrate=%dkbps",
             uvc_dev_path, uvc_width, uvc_height, uvc_fps,
             fmt_name, encoder->name, bitrate_kbps);
    return 0;
}

/* runtime re-init helpers removed */

int uvc_start_streaming() {
    pthread_mutex_lock(&uvc_mutex);
    if (uvc_running) { pthread_mutex_unlock(&uvc_mutex); return 0; }
    uvc_warmup_frames = uvc_warmup_frames_default;
    uvc_running = 1;
    int rc = pthread_create(&uvc_thread, NULL, uvc_thread_main, NULL);
    if (rc != 0) { uvc_running = 0; log_error("UVC: pthread_create failed: %s", strerror(rc)); }
    if (rc == 0) {
        uvc_thread_started = 1;
    }
    pthread_mutex_unlock(&uvc_mutex);
    return rc == 0 ? 0 : -1;
}

void uvc_stop_streaming() {
    pthread_mutex_lock(&uvc_mutex);
    if (!uvc_running && !uvc_thread_started) { pthread_mutex_unlock(&uvc_mutex); return; }
    uvc_running = 0;
    pthread_mutex_unlock(&uvc_mutex);
    if (uvc_thread_started) {
        // Wake the thread by select timeout; then join
        pthread_join(uvc_thread, NULL);
        pthread_mutex_lock(&uvc_mutex);
        uvc_thread_started = 0;
        pthread_mutex_unlock(&uvc_mutex);
    }
}

int uvc_is_streaming() {
    int running;
    pthread_mutex_lock(&uvc_mutex);
    running = uvc_running;
    pthread_mutex_unlock(&uvc_mutex);
    return running;
}

void uvc_shutdown() {
    if (!uvc_initialized) {
        log_warn("UVC: shutdown called while not initialized");
        return;
    }

    uvc_stop_streaming();
    
    // 统一销毁编码器
    if (encoder && encoder->destroy) {
        encoder->destroy(&encoder_ctx);
        encoder = NULL;
        encoder_ctx = NULL;
    }
    
    // 销毁解码器
#ifdef JETKVM_HAVE_MPP
    if (use_mpp_decoder && uvc_pixfmt == V4L2_PIX_FMT_MJPEG) {
        dec_mppjpeg_deinit();
    }
#endif
    if (tj) { 
        tjDestroy(tj); 
        tj = NULL; 
    }
    
    // 释放缓冲区
    if (yuv_buf) { free(yuv_buf); yuv_buf = NULL; }
    if (yuv_src) { free(yuv_src); yuv_src = NULL; }
    yuv_buf_size = 0;
    yuv_src_size = 0;
    mjpeg_subsamp = -1;
    mjpeg_colorspace = -1;
    tj_fail_count = 0;
    uvc_warmup_frames = 0;
    
    // 关闭V4L2设备
    if (uvc_fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        xioctl(uvc_fd, VIDIOC_STREAMOFF, &type);
        for (int i = 0; i < uvc_nbufs; i++) {
            if (uvc_buffers[i].start && uvc_buffers[i].length) {
                munmap(uvc_buffers[i].start, uvc_buffers[i].length);
            }
        }
        free(uvc_buffers); 
        uvc_buffers = NULL; 
        uvc_nbufs = 0;
        close(uvc_fd); 
        uvc_fd = -1;
    }

    uvc_initialized = 0;
    
    log_info("UVC: shutdown completed");
}

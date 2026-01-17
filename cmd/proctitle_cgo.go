//go:build cgo

package main

import (
	"fmt"

	"github.com/erikdubbelboer/gspt"
)

func setProcTitle(status string) {
	if status != "" {
		status = " " + status
	}
	title := fmt.Sprintf("jetkvm: [supervisor]%s", status)
	gspt.SetProcTitle(title)
}

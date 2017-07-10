# README #

This package implements simple image dilation detection in a streaming sequence
of camera images. The detection works by comparing a current frame to dilations
of the previous frame at various scales.

This kind of detection can provide a coarse estimate of motion flow in a scene
that is directed either toward or away from the camera. The emphasis is on
reactivity rather than accuracy, so the library is designed to be as fast and
lightweight as is reasonable.

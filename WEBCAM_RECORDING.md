# Webcam Recording with Audio (FFmpeg)

This directory contains a script for recording video + audio from your webcam using FFmpeg.

## Prerequisites

Install FFmpeg if not already installed:
```bash
sudo apt-get update
sudo apt-get install ffmpeg
```

## Usage

### Quick Start
```bash
# Record with automatic timestamp filename
./record_webcam.sh

# Record with custom filename
./record_webcam.sh my_video.mp4
```

Press **Ctrl+C** to stop recording.

### Find Your Audio Device

List available audio devices:
```bash
arecord -l
```

Example output:
```
card 0: PCH [HDA Intel PCH], device 0: ALC269VC Analog [ALC269VC Analog]
card 1: U0x46d0x825 [USB Device 0x46d:0x825], device 0: USB Audio [USB Audio]
```

If your webcam has a microphone and shows as card 1, edit `record_webcam.sh` and change:
```bash
AUDIO_DEVICE="hw:1,0"  # Use card 1 instead of card 0
```

### Find Your Video Device

List video devices:
```bash
v4l2-ctl --list-devices
```

If your webcam is not `/dev/video0`, edit `record_webcam.sh` and update:
```bash
VIDEO_DEVICE="/dev/video2"  # Example: use video2
```

## Testing

Test if FFmpeg can access your webcam and microphone:
```bash
# Test video only (5 seconds)
ffmpeg -f v4l2 -i /dev/video0 -t 5 test_video.mp4

# Test audio only (5 seconds)
ffmpeg -f alsa -i hw:0,0 -t 5 test_audio.wav

# Test video + audio (5 seconds)
ffmpeg -f v4l2 -i /dev/video0 -f alsa -i hw:0,0 -t 5 test.mp4
```

## Using with Vlogger System

You can run the webcam recording in parallel with the vlogger system:

**Terminal 1 (Recording):**
```bash
./record_webcam.sh
```

**Terminal 2 (Vlogger with Face Tracking):**
```bash
source install/setup.bash
ros2 run vlogger_system vlogger_control
```

This records the raw webcam feed with audio, while the vlogger system controls the robot arm to track your face.

## Output

- Videos are saved to: `recordings/webcam_YYYYMMDD_HHMMSS.mp4`
- Format: MP4 with H.264 video and AAC audio
- Resolution: 640x480 @ 30 FPS
- Audio: 128 kbps AAC

## Troubleshooting

### "No such file or directory" for audio device
Your audio device might be different. Run `arecord -l` and update `AUDIO_DEVICE` in the script.

### "Device or resource busy"
Another application is using the webcam. Close any camera applications and try again.

### No audio in recording
- Check microphone is not muted: `alsamixer`
- Test microphone: `arecord -d 3 test.wav && aplay test.wav`
- Verify correct audio device in script

### Poor video quality
Edit the script and change the CRF value (lower = better quality):
```bash
-crf 18  # High quality (larger file)
-crf 23  # Default (balanced)
-crf 28  # Lower quality (smaller file)
```

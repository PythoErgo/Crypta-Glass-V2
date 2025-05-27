'''
Pytho Industries
27/05/2025
CryptaGlassV2 Code (Release 1)

Comments mainly created with ChatGPT, please any report issues in CryptaGlassV2 Instructables tutorial comment section.

Bug fixes and improvements will begin within 2 weeks.
This week is exam week...
'''

from PIL import Image, ImageDraw, ImageFont, ImageOps  # image manipulation and fonts
from luma.core.interface.serial import spi  # SPI interface for display communication
from luma.lcd.device import st7735  # LCD driver for ST7735 display
import RPi.GPIO as GPIO  # GPIO pin control
from adafruit_ads1x15.analog_in import AnalogIn  # ADC channel reading
import adafruit_ads1x15.ads1115 as ADS  # ADC driver
import busio  # I2C communication
import board  # board pin definitions
import cv2  # OpenCV for camera and image processing
import time  # timing functions
import os  # OS commands
import bluetooth  # Bluetooth sockets
import threading  # threading for concurrency
import multiprocessing # use multiple cores simultaneously
from picamera2 import Picamera2, Preview  # camera control
from libcamera import Transform  # camera image transformations
from picamera2.encoders import H264Encoder  # video encoding
from datetime import datetime  # timestamp handling
import numpy as np  # numerical operations
from collections import deque  # efficient queue structure
import subprocess  # running subprocess commands
import vlc  # media playback
import signal  # signal handling

# Display setup over SPI interface
serial = spi(
    port=0,  # SPI bus port 0
    device=0,  # SPI device 0 (CS0)
    gpio_DC=24,  # GPIO pin for data/command control
    gpio_RST=25,  # GPIO pin for hardware reset
    bus_speed_hz=8000000  # SPI clock speed 8 MHz
)

# Initialize ST7735 LCD with given dimensions and settings
display = st7735(
    serial,
    width=160,
    height=80,
    bgr=True,  # color ordering is BGR
    inverse=True,  # invert colors for display
    rotate=0,  # no rotation
    h_offset=1,  # horizontal pixel offset
    v_offset=26  # vertical pixel offset to align display area
)

# Load font with fallback to default if loading fails
try:
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    font = ImageFont.truetype(font_path, size=10)  # custom font size 10
except:
    font = ImageFont.load_default()  # fallback to default font

# GPIO button setup using BCM numbering and pull-up resistors
GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button Up with pull-up
GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button Select with pull-up
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button Down with pull-up

def wait_for_button_release(pin):
    # Loop until button released (input HIGH)
    while GPIO.input(pin) == GPIO.LOW:
        time.sleep(0.01)  # short wait to avoid busy loop
    time.sleep(0.1)  # debounce delay to avoid false triggers

# Battery voltage thresholds for percentage calculation
V_FULL = 4.2  # voltage when battery is full
V_EMPTY = 3.2  # voltage when battery is empty

adc_available = False  # flag to track ADC initialization
ads = None  # ADC device object
adc_channel = None  # specific ADC input channel

def try_initialize_adc():
    global adc_available, ads, adc_channel
    if not adc_available:
        try:
            i2c = busio.I2C(board.SCL, board.SDA)  # set up I2C bus
            ads = ADS.ADS1115(i2c)  # create ADS1115 ADC object
            ads.gain = 2/3  # gain setting to read full voltage range
            adc_channel = AnalogIn(ads, ADS.P0)  # use channel 0 on ADC
            adc_available = True
        except Exception:
            adc_available = False  # ADC setup failed

def voltage_to_percentage(voltage):
    voltage = max(min(voltage, V_FULL), V_EMPTY)  # clamp voltage to valid range
    percent = ((voltage - V_EMPTY) / (V_FULL - V_EMPTY)) * 100  # linear mapping
    return round(percent / 5) * 5  # round to nearest multiple of 5

# Bluetooth socket placeholders
server_sock = None
client_sock = None

def bluetooth_listener():
    global latest_notification, latest_navigation, server_sock, client_sock
    try:
        os.system("sudo sdptool add SP")  # register serial port profile (SPP) on channel 1
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)  # create RFCOMM socket
        port = 1  # RFCOMM uses channel 1 by convention
        server_sock.bind(("", port))
        server_sock.listen(1)  # start listening for client connection
        print("Waiting for connection on RFCOMM channel 1...")

        client_sock, address = server_sock.accept()  # block until connection
        print(f"Connected to {address}.")

        while True:
            data = client_sock.recv(1024)  # receive up to 1024 bytes
            if not data:
                break  # connection closed by client
            message = data.decode("utf-8").strip()
            if message.startswith("Google Maps: "):
                with navigation_lock:  # safely update shared navigation data
                    latest_navigation = message[len("Google Maps: "):].strip()
                print(f"Received navigation: {latest_navigation}")
            else:
                with notification_lock:  # safely update shared notification data
                    latest_notification = message
                print(f"Received: {latest_notification}")

    except Exception as e:
        print(f"Bluetooth error: {e}")

def close_bluetooth():
    # safely close any open Bluetooth sockets
    if client_sock:
        client_sock.close()
    if server_sock:
        server_sock.close()

    print("Attempting to remove Bluetooth services on channel 1...")
    try:
        # parse SDP service records to find services on channel 1
        services = os.popen("sudo sdptool browse local").read()
        lines = services.splitlines()
        handles = []
        current_handle = None
        for line in lines:
            line = line.strip()
            if line.startswith("Service RecHandle:"):
                current_handle = line.split(":")[1].strip()
            if "Channel: 1" in line and current_handle:
                handles.append(current_handle)
                current_handle = None

        for handle in handles:
            os.system(f"sudo sdptool del {handle}")  # remove each service by handle
            print(f"Removed service with handle {handle}.")
        if not handles:
            print("No services found on channel 1.")
    except Exception as e:
        print(f"Failed to remove services: {e}")

    print("Bluetooth sockets and services cleaned up.")

# === Main Menu Page ===
# Menu options list shown on display
menu_items = [
    "Notifications", "Call", "Record Video", "Take Photo",
    "Music Player", "Weather", "Navigator", "Date & Time", "Shutdown"
]
selected_index = 0  # index of currently selected menu item

def draw_menu():
    global selected_index
    try_initialize_adc()  # ensure ADC is initialized
    visible_lines = 4  # max number of menu items shown at once
    scroll_top = max(0, selected_index - visible_lines + 1) if selected_index >= visible_lines else 0  # determine scroll start index

    img = Image.new("RGB", (160, 80), "black")  # create empty image buffer
    draw = ImageDraw.Draw(img)

    title = "CryptaGlass V2"
    draw.text((2, 2), title, font=font, fill="red")  # draw title in top left

    try:
        if adc_available:
            voltage = adc_channel.voltage  # read current battery voltage
            if voltage < 3.0:
                battery_status = "USB IN"  # assume charging if voltage too low
            else:
                percent = voltage_to_percentage(voltage)
                battery_status = f"{percent}%"
        else:
            battery_status = "USB IN"  # fallback if ADC not available
    except Exception:
        battery_status = "USB IN"  # fallback on read error

    status_width = draw.textlength(battery_status, font=font)
    draw.text((160 - status_width - 2, 2), battery_status, font=font, fill="red")  # draw battery status top right

    for i in range(visible_lines):
        idx = scroll_top + i
        if idx >= len(menu_items):
            break  # end of menu
        prefix = "> " if idx == selected_index else "  "  # highlight selected item
        line = prefix + menu_items[idx]
        y_pos = 18 + i * 14  # vertical spacing per line
        draw.text((10, y_pos), line, font=font, fill="red")

    display.display(img)  # render menu to physical display
    
# === Notifications Page ===
# Latest notification text storage with threading lock for safe updates
latest_notification = "No new notifications."
notification_lock = threading.Lock()

def draw_notifications_page(notification_text, scroll_index):
    img = Image.new("RGB", (160, 80), "black")
    draw = ImageDraw.Draw(img)

    # Centered title
    title = "Notifications"
    title_width = draw.textlength(title, font=font)
    draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

    # Wrap notification text into multiple lines fitting display width (140 px)
    lines = []
    words = notification_text.split()
    current_line = ""
    for word in words:
        test_line = current_line + word + " "
        if draw.textlength(test_line, font=font) < 140:  # check line width
            current_line = test_line
        else:
            lines.append(current_line.strip())  # line fits limit, add to list
            current_line = word + " "
    if current_line:
        lines.append(current_line.strip())  # add any remaining text

    # Draw up to 4 lines starting from scroll_index to allow vertical scrolling
    for i in range(4):
        idx = scroll_index + i
        if idx < len(lines):
            draw.text((10, 18 + i * 14), lines[idx], font=font, fill="red")

    display.display(img)
    return len(lines)  # return total number of wrapped lines

def show_notifications():
    global running_notifications
    running_notifications = True

    scroll_index = 0
    previous_text = ""
    last_scroll_time = time.time()

    while running_notifications:
        with notification_lock:
            current_notification = latest_notification  # safely get latest text

        if current_notification != previous_text:
            scroll_index = 0  # reset scroll on text change
            previous_text = current_notification

        if time.time() - last_scroll_time >= 5:  # scroll every 5 seconds
            line_count = draw_notifications_page(current_notification, scroll_index)
            scroll_index = (scroll_index + 1) % max(1, line_count - 3)  # wrap around if needed
            last_scroll_time = time.time()

        # Exit if SELECT button pressed
        if GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)
            running_notifications = False

        time.sleep(0.1)  # reduce CPU usage

    draw_menu()  # return to main menu

# === Call Page ===
# Contacts list for call menu including a "Return" option
contacts = ["Caller1", "Caller2", "etc", "Return"]
call_selected_index = 0  # currently highlighted contact index

def show_call_menu():
    global call_selected_index
    visible_lines = 4  # max lines visible on screen

    while True:
        # adjust top of scroll window if selection goes beyond visible area
        scroll_top = max(0, call_selected_index - visible_lines + 1) if call_selected_index >= visible_lines else 0

        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)
        title = "Call Menu"
        title_width = draw.textlength(title, font=font)
        draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

        for i in range(visible_lines):
            idx = scroll_top + i
            if idx >= len(contacts):
                break
            prefix = "> " if idx == call_selected_index else "  "  # highlight selected line
            line = prefix + contacts[idx]
            y_pos = 18 + i * 14
            draw.text((10, y_pos), line, font=font, fill="red")

        display.display(img)

        # Navigation buttons
        if GPIO.input(21) == GPIO.LOW:  # UP
            wait_for_button_release(21)
            call_selected_index = (call_selected_index - 1) % len(contacts)

        elif GPIO.input(12) == GPIO.LOW:  # DOWN
            wait_for_button_release(12)
            call_selected_index = (call_selected_index + 1) % len(contacts)

        elif GPIO.input(16) == GPIO.LOW:  # SELECT
            wait_for_button_release(16)
            selected_contact = contacts[call_selected_index]
            if selected_contact.lower() == "return":
                print("Returning to main menu.")
                break
            else:
                message = f"Call {selected_contact}"
                if client_sock:
                    try:
                        client_sock.send(message + "\n")  # send call command
                        print(f"Sent: {message}")
                    except Exception as e:
                        print(f"Failed to send message: {e}")
                else:
                    print("Bluetooth connection not available.")

                # show confirmation screen
                img = Image.new("RGB", (160, 80), "black")
                draw = ImageDraw.Draw(img)
                call_msg = f"Calling {selected_contact}."
                msg_width = draw.textlength(call_msg, font=font)
                draw.text(((160 - msg_width) // 2, 35), call_msg, font=font, fill="red")
                display.display(img)
                time.sleep(2)

        time.sleep(0.1)  # reduce CPU usage

# === Record Video Page ===
video_resolutions = [
    ("1080x1920 30fps", {"size": (1080, 1920), "framerate": 30}),  # portrait high res fixed to 30fps
    ("720x1280 60fps", {"size": (720, 1280), "framerate": 60}),   # medium res higher fps option
    ("480x640 10fps", {"size": (480, 640), "framerate": 10}),     # low res low fps for testing
    ("Return", None)  # menu exit option
]

video_selected_index = 0
is_recording = False
framerate = 30
video_filename = "/home/pi/video.h264"
audio_wav_filename = "/home/pi/audio.wav"
final_filename = ""
picam = None
audio_process = None
audio_thread = None
video_thread = None

# note audio recording currently does not work properly will be fixed soon video recording unaffected
def record_audio():
    global audio_process, is_recording
    try:
        print("Starting audio recording")
        audio_process = subprocess.Popen([
            "arecord", "-D", "dmic_sv", "-c", "2", "-r", "48000",
            "-f", "S32_LE", "-t", "wav", "-q", "audio.wav"
        ])  # start audio capture with mic device
        while is_recording and audio_process.poll() is None:
            time.sleep(0.1)
        if audio_process.poll() is None:
            print("Sending SIGINT to arecord to stop recording gracefully")
            audio_process.send_signal(signal.SIGINT)
            try:
                audio_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("Terminating arecord forcefully due to timeout")
                audio_process.terminate()  # fallback stop if hang
        print("Audio recording thread exited cleanly")
    except Exception as e:
        print(f"Audio thread error: {e}")

def record_video(config):
    global is_recording, framerate, video_filename, picam
    try:
        requested_size = config["size"]
        framerate = config["framerate"]
        # swap width and height for portrait mode due to horizontal camera mount
        swapped_size = (requested_size[1], requested_size[0])
        print(f"Starting video at {swapped_size} @ {framerate}fps")

        if picam:
            picam.stop()  # stop preview before reconfiguring

        picam.set_controls({"FrameRate": framerate, "NoiseReductionMode": 0})
        video_config = picam.create_video_configuration({"size": swapped_size, "format": "YUV420"})
        picam.configure(video_config)

        encoder = H264Encoder()
        picam.start_recording(encoder, video_filename)  # begin video capture
        while is_recording:
            time.sleep(0.1)

        picam.stop_recording()  # stop when recording flag cleared
        print("Video recording stopped")

    except Exception as e:
        print(f"Video thread error: {e}")
        
def recording_status(message):
    img = Image.new("RGB", (160, 80), "black")  # create status image buffer
    draw = ImageDraw.Draw(img)
    text_width = draw.textlength(message, font=font)
    draw.text(((160 - text_width) // 2, 32), message, font=font, fill="red")  # center status text
    display.display(img)

def start_recording(config):
    global is_recording, audio_thread, video_thread
    is_recording = True
    recording_status("Recording...")  # update display
    
    audio_thread = threading.Thread(target=record_audio, daemon=True)
    video_thread = threading.Thread(target=record_video, args=(config,), daemon=True)
    audio_thread.start()
    video_thread.start()

def stop_recording():
    global is_recording
    is_recording = False  # signal threads to stop
    recording_status("Muxing...")  # show muxing status
    
    if audio_thread:
        audio_thread.join()
    if video_thread:
        video_thread.join()

def show_take_video_menu():
    global video_selected_index, is_recording, picam
    try:
        picam = Picamera2()
        picam.configure(picam.create_still_configuration())  # setup preview before video mode
        picam.start()
        time.sleep(2)  # warm up camera sensor
    except Exception as e:
        print(f"Camera initialization failed: {e}")
        return

    while True:
        if not is_recording:
            # draw resolution selection UI
            img = Image.new("RGB", (160, 80), "black")
            draw = ImageDraw.Draw(img)
            title = "Take Video"
            title_width = draw.textlength(title, font=font)
            draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

            for i, (label, _) in enumerate(video_resolutions):
                prefix = "> " if i == video_selected_index else "  "
                draw.text((10, 18 + i * 14), prefix + label, font=font, fill="red")
            display.display(img)

        # handle button input for menu navigation
        if GPIO.input(21) == GPIO.LOW:  # UP button
            wait_for_button_release(21)
            video_selected_index = (video_selected_index - 1) % len(video_resolutions)

        elif GPIO.input(12) == GPIO.LOW:  # DOWN button
            wait_for_button_release(12)
            video_selected_index = (video_selected_index + 1) % len(video_resolutions)

        elif GPIO.input(16) == GPIO.LOW:  # SELECT button
            wait_for_button_release(16)
            label, config = video_resolutions[video_selected_index]

            if is_recording:
                stop_recording()
                mux_audio_video()
                continue

            if label.lower() == "return":
                if is_recording:
                    stop_recording()
                    mux_audio_video()
                print("Exiting video menu")
                picam.close()
                return

            if config:
                start_recording(config)

        time.sleep(0.1)  # button debounce delay

def mux_audio_video():
    global final_filename, framerate
    timestamp = time.strftime("%Y%m%d_%H%M%S")  # generate unique filename
    final_filename = f"/home/pi/Videos/recording_{timestamp}.mp4"
    subprocess.run([
        "ffmpeg", "-y",
        "-framerate", str(framerate),
        "-i", video_filename,
        "-i", audio_wav_filename,
        "-metadata:s:v", "rotate=270",  # rotate video to portrait orientation
        "-c:v", "copy",
        "-c:a", "aac",
        "-shortest", final_filename
    ], check=True)  # mux audio and video into mp4 container
    # note final video's fps metadata always shows 30fps this will be fixed soon playback unaffected
    print(f"Recording saved as {final_filename}")
                
# === Take Photo Page ===
# List of photo resolutions and labels last option returns to menu
photo_resolutions = [("1944x2592", (2592, 1944)), ("1080p portrait", (1920, 1080)), ("720p portrait", (1280, 720)), ("Return", None)]
photo_selected_index = 0  # currently highlighted menu item

def show_take_photo_menu():
    global photo_selected_index

    picam = Picamera2()  # initialize camera

    while True:
        # Create blank black image for menu
        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)
        title = "Take Photo"
        title_width = draw.textlength(title, font=font)
        # Draw centered title at top
        draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

        # Draw resolution options highlight selected with '> '
        for i, (label, _) in enumerate(photo_resolutions):
            prefix = "> " if i == photo_selected_index else "  "
            line = prefix + label
            y_pos = 18 + i * 14  # vertical spacing
            draw.text((10, y_pos), line, font=font, fill="red")

        display.display(img)  # show menu on display

        # Button input to navigate and select
        if GPIO.input(21) == GPIO.LOW:  # UP button
            wait_for_button_release(21)
            photo_selected_index = (photo_selected_index - 1) % len(photo_resolutions)  # wrap around menu
        elif GPIO.input(12) == GPIO.LOW:  # DOWN button
            wait_for_button_release(12)
            photo_selected_index = (photo_selected_index + 1) % len(photo_resolutions)
        elif GPIO.input(16) == GPIO.LOW:  # SELECT button
            wait_for_button_release(16)
            label, resolution = photo_resolutions[photo_selected_index]
            if label.lower() == "return":
                print("Returning")  # exit photo menu
                return

            # Restart camera with selected resolution config
            try:
                picam.stop()  # stop any prior config
            except Exception:
                pass

            picam.configure(picam.create_video_configuration(
                main={"size": resolution, "format": "RGB888"},
                controls={"FrameRate": 10}  # low fps for preview only
            ))
            picam.start()
            time.sleep(1)  # camera warm-up delay

            print(f"[INFO] Previewing at {label}")
            preview_and_capture(picam, label, resolution)  # enter preview/capture loop

        time.sleep(0.1)  # debounce

def preview_and_capture(picam, label, resolution):
    preview_active = True
    
    fps_deque = deque(maxlen=30)  # timestamps to compute FPS

    while preview_active:
        frame = picam.capture_array("main")  # capture frame
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)  # rotate for correct orientation

        # Calculate aspect ratio (width / height) for portrait, usually < 1
        aspect_ratio = resolution[0] / resolution[1]  # width divided by height

        target_height = 80  # fixed display height
        target_width = int(target_height * aspect_ratio)  # keep aspect ratio correct

        # Resize frame to fit display height keeping aspect ratio
        frame_resized = cv2.resize(frame, (target_width, target_height))

        # Create black image with full display width and pad resized frame horizontally centered
        padding_left = (160 - target_width) // 2
        frame_padded = np.zeros((target_height, 160, 3), dtype=np.uint8)
        frame_padded[:, padding_left:padding_left + target_width] = frame_resized

        # Calculate FPS from timestamps of last frames
        now = time.time()
        fps_deque.append(now)
        elapsed = now - fps_deque[0] if len(fps_deque) > 1 else 0
        fps = len(fps_deque) / elapsed if elapsed > 0 else 0

        # Convert BGR(OpenCV) to RGB(PIL) for display
        img_pil = Image.fromarray(cv2.cvtColor(frame_padded, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        fps_text = f"{fps:.1f} FPS"
        text_width = draw.textlength(fps_text, font=font)
        # Draw FPS in top-right corner
        draw.text((160 - text_width - 2, 2), fps_text, font=font, fill="red")
        display.display(img_pil)  # update display with preview

        # Buttons: SELECT to capture, UP or DOWN to exit preview
        if GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)
            print("[INFO] Capturing photo")

            # Freeze last frame in grayscale for feedback
            gray_frame = cv2.cvtColor(frame_padded, cv2.COLOR_BGR2GRAY)
            gray_rgb = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)
            img = Image.fromarray(gray_rgb)

            try:
                still = picam.capture_array("main")  # capture final photo
                still = cv2.rotate(still, cv2.ROTATE_90_CLOCKWISE)
            except Exception as e:
                print(f"[ERROR] Failed to capture photo: {e}")
                continue  # resume preview if capture fails

            # Save photo with timestamp and resolution label
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/home/pi/Pictures/photo_{timestamp}_{label}.jpg"
            cv2.imwrite(filename, still)

            print(f"[INFO] Photo saved as {filename}")

            display.display(img)  # show freeze frame feedback
            time.sleep(2)  # hold feedback 2 seconds

        elif GPIO.input(21) == GPIO.LOW or GPIO.input(12) == GPIO.LOW:
            # Return to menu if UP or DOWN pressed
            if GPIO.input(21) == GPIO.LOW:
                wait_for_button_release(21)
            if GPIO.input(12) == GPIO.LOW:
                wait_for_button_release(12)
            print("Returning to menu")
            break  # exit preview loop

        time.sleep(0.1)  # debounce

    picam.stop()  # stop camera before returning
    
# === Music Player Page ===
# Initialize VLC instance and media player variables
_instance = None  # holds VLC instance for playback control
_player = None    # VLC media player object

def init_audio():
    global _instance, _player
    _instance = vlc.Instance(
        '--no-video',           # disable video output for audio only
        '--no-xlib',            # prevent X window usage (headless)
        '--file-caching=3000',  # cache files for 3000ms to reduce stutter
        '--network-caching=3000', # cache network streams similarly
        '--aout=alsa',          # use ALSA audio output backend
        '--alsa-audio-device=default' # use default ALSA device
    )
    _player = _instance.media_player_new()  # create new media player
    print("VLC audio system initialized.")
    
init_audio()

def play_song_on_bluetooth(song_path, volume=1.0, loop=False):
    global _player
    if _player.is_playing():
        _player.stop()  # stop current playback before new one

    media = _instance.media_new(song_path)  # create new media from file path
    _player.set_media(media)                 # assign media to player
    _player.audio_set_volume(int(volume * 100))  # set volume (0-100 scale)

    def play():
        _player.play()
        print(f"Now playing: {song_path}")
        # wait briefly until playback starts (or timeout)
        for _ in range(20):
            if _player.get_state() == vlc.State.Playing:
                break
            time.sleep(0.1)

    # run playback in background thread so UI doesn't block
    threading.Thread(target=play, daemon=True).start()

def stop_song():
    global _player
    if _player:
        _player.stop()  # stop playback immediately
        print("Music stopped.")
    
def adjust_volume(volume):
    global _player
    volume = min(max(volume, 0.0), 1.0)  # clamp volume between 0 and 1
    _player.audio_set_volume(int(volume * 100))  # VLC expects 0-100 scale
    print(f"Volume set to: {volume * 100:.0f}%.")

# Menu items shown on display
music_menu_items = ["Choose Song", "Choose Playlist", "Connect Bluetooth", "Return"]
music_selected_index = 0

def draw_music_player_menu():
    global music_selected_index
    img = Image.new("RGB", (160, 80), "black")  # blank display image with black bg
    draw = ImageDraw.Draw(img)
    # center and draw menu title
    draw.text(((160 - draw.textlength("Music Player", font)) // 2, 2),
              "Music Player", font=font, fill="red")
    # scrolling offset if selected item beyond first 4
    scroll_top = max(0, music_selected_index - 3) if music_selected_index >= 4 else 0
    for i in range(4):
        idx = scroll_top + i
        if idx >= len(music_menu_items): break
        prefix = "> " if idx == music_selected_index else "  "  # highlight selected item
        draw.text((10, 18 + i*14),
                  prefix + music_menu_items[idx], font=font, fill="red")
    display.display(img)  # update the physical display

def show_music_player():
    global music_selected_index
    while True:
        draw_music_player_menu()
        # check button inputs for navigation and selection
        if GPIO.input(21) == GPIO.LOW:  # UP button pressed
            wait_for_button_release(21)
            music_selected_index = (music_selected_index - 1) % len(music_menu_items)
        elif GPIO.input(12) == GPIO.LOW:  # DOWN button pressed
            wait_for_button_release(12)
            music_selected_index = (music_selected_index + 1) % len(music_menu_items)
        elif GPIO.input(16) == GPIO.LOW:  # SELECT button pressed
            wait_for_button_release(16)
            opt = music_menu_items[music_selected_index].lower()
            # call function based on selected option
            if opt == "choose song":
                choose_song()
            elif opt == "choose playlist":
                choose_playlist()
            elif opt == "connect bluetooth":
                connect_bluetooth_devices()
            elif opt == "return":
                break  # exit music menu loop
        time.sleep(0.05)  # small delay to reduce CPU usage
    draw_menu()  # redraw main menu after exit

def get_music_files():
    music_dir = "/home/pi/Music"
    try:
        # list audio files with common extensions only
        return sorted(f for f in os.listdir(music_dir)
                      if f.lower().endswith((".mp3", ".wav", ".flac")))
    except Exception as e:
        print(f"Error reading music directory: {e}")
        return []

def choose_song():
    songs = get_music_files()
    if not songs:
        return  # no songs found, exit function
    songs.append("Return")  # add return option at end
    song_selected = 0
    while True:
        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)
        # draw menu title centered
        draw.text(((160 - draw.textlength("Choose Song", font)) // 2, 2),
                  "Choose Song", font=font, fill="red")
        top = max(0, song_selected - 3)
        for i in range(4):
            idx = top + i
            if idx >= len(songs): break
            prefix = "> " if idx == song_selected else "  "
            name = os.path.basename(songs[idx])[:15]  # truncate long names
            draw.text((10, 18 + i*14), prefix + name, font=font, fill="red")
        display.display(img)

        # navigation buttons for song selection
        if GPIO.input(21) == GPIO.LOW:
            wait_for_button_release(21)
            song_selected = (song_selected - 1) % len(songs)
        elif GPIO.input(12) == GPIO.LOW:
            wait_for_button_release(12)
            song_selected = (song_selected + 1) % len(songs)
        elif GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)
            if songs[song_selected] == "Return":
                return  # exit to previous menu
            # play selected song, pass full file paths
            play_song_ui([os.path.join("/home/pi/Music", s) for s in songs[:-1]], song_selected)
            return
        time.sleep(0.05)

def get_music_folders():
    music_dir = "/home/pi/Music"
    try:
        # list directories only for playlists
        return sorted(f for f in os.listdir(music_dir)
                      if os.path.isdir(os.path.join(music_dir, f)))
    except Exception as e:
        print(f"Error reading playlist folders: {e}")
        return []

def choose_playlist():
    folders = get_music_folders()
    if not folders:
        return  # no playlists found
    folders.append("Return")
    selected = 0
    while True:
        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)
        draw.text(((160 - draw.textlength("Choose Playlist", font)) // 2, 2),
                  "Choose Playlist", font=font, fill="red")
        top = max(0, selected - 3)
        for i in range(4):
            idx = top + i
            if idx >= len(folders): break
            prefix = "> " if idx == selected else "  "
            name = folders[idx][:15]
            draw.text((10, 18 + i*14), prefix + name, font=font, fill="red")
        display.display(img)

        # navigation buttons for playlist selection
        if GPIO.input(21) == GPIO.LOW:
            wait_for_button_release(21)
            selected = (selected - 1) % len(folders)
        elif GPIO.input(12) == GPIO.LOW:
            wait_for_button_release(12)
            selected = (selected + 1) % len(folders)
        elif GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)
            if folders[selected] == "Return":
                return  # exit to previous menu
            folder_path = os.path.join("/home/pi/Music", folders[selected])
            # list audio files inside selected playlist folder
            songs = sorted(f for f in os.listdir(folder_path)
                           if f.lower().endswith((".mp3", ".wav", ".flac")))
            songs = [os.path.join(folder_path, s) for s in songs]  # absolute paths
            if songs:
                play_song_ui(songs, 0, is_playlist=True)
            return
        time.sleep(0.05)

def play_song_ui(songs, song_index, is_playlist=False):
    global _player
    is_looping = False
    is_paused = False
    volume = 1.0

    # control icons with labels and positions on display
    icons = [
        {"action": "exit", "label": "X", "x": 5, "y": 2},
        {"action": "prev", "label": "<", "x": 5, "y": 40},
        {"action": "pause_play", "label": "▶", "x": 50, "y": 40},
        {"action": "loop", "label": "L", "x": 90, "y": 40},
        {"action": "next", "label": ">", "x": 140, "y": 40},
        {"action": "vol_down", "label": "-", "x": 10, "y": 60},
        {"action": "vol_up", "label": "+", "x": 140, "y": 60},
    ]

    play_song_on_bluetooth(songs[song_index], volume, is_looping)  # start playback

    def loop_checker():
        nonlocal song_index
        while True:
            state = _player.get_state()
            if state == vlc.State.Ended:
                if is_looping:
                    _player.stop()
                    _player.play()  # replay current song if looping enabled
                elif is_playlist:
                    # advance to next song if playlist and not looping
                    song_index = (song_index + 1) % len(songs)
                    play_song_on_bluetooth(songs[song_index], volume, is_looping)
            time.sleep(1)  # check every second

    # thread to monitor song end and handle looping or next song
    threading.Thread(target=loop_checker, daemon=True).start()

    selected_icon = 0
    while True:
        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)

        # display truncated song name
        song_name = os.path.basename(songs[song_index])[:20]
        draw.text((25, 2), song_name, font=font, fill="red")

        # draw control icons, highlight selected, color changes for states
        for idx, ico in enumerate(icons):
            is_selected = (idx == selected_icon)
            act = ico["action"]
            # show play symbol if paused else pause symbol otherwise
            lbl = "▶" if act == "pause_play" and is_paused else "II" if act == "pause_play" else ico["label"]
            # loop icon green if active, white if selected, else red
            if act == "loop":
                col = (0, 255, 0) if is_looping else ("white" if is_selected else "red")
            else:
                col = "white" if is_selected else "red"
            draw.text((ico["x"], ico["y"]), lbl, font=font, fill=col)

        # display volume percentage centered at bottom
        vol_text = f"{int(volume * 100)}%"
        x_vol = (160 - draw.textlength(vol_text, font=font)) // 2
        draw.text((x_vol, 60), vol_text, font=font, fill="red")

        display.display(img)

        # navigate icons with buttons
        if GPIO.input(21) == GPIO.LOW:
            wait_for_button_release(21)
            selected_icon = (selected_icon - 1) % len(icons)
        elif GPIO.input(12) == GPIO.LOW:
            wait_for_button_release(12)
            selected_icon = (selected_icon + 1) % len(icons)
        elif GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)
            act = icons[selected_icon]["action"]
            # handle icon action commands
            if act == "exit":
                stop_song()
                return
            elif act == "pause_play":
                if is_paused:
                    _player.play()
                else:
                    _player.pause()
                is_paused = not is_paused
            elif act == "loop":
                is_looping = not is_looping  # toggle loop mode
            elif act == "prev":
                if not is_paused:
                    song_index = (song_index - 1) % len(songs)
                    play_song_on_bluetooth(songs[song_index], volume, is_looping)
            elif act == "next":
                if not is_paused:
                    song_index = (song_index + 1) % len(songs)
                    play_song_on_bluetooth(songs[song_index], volume, is_looping)
            elif act == "vol_down":
                volume = max(0, volume - 0.1)
                adjust_volume(volume)
            elif act == "vol_up":
                volume = min(1, volume + 0.1)
                adjust_volume(volume)

        time.sleep(0.1)  # small delay to reduce CPU load

# Note wav files work fine mp3 playback is choppy not recommended will fix soon

def scan_bluetooth_devices():
    """Scan for Bluetooth devices and return a list of (address, name) tuples"""
    print("Scanning for Bluetooth devices...")
    # Start bluetoothctl scanning process, suppress output
    scan_proc = subprocess.Popen("bluetoothctl scan on", shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5)  # Wait 5 seconds for devices to be discovered
    subprocess.run("bluetoothctl scan off", shell=True)  # Stop scanning to conserve resources

    # Get list of detected devices from bluetoothctl
    result = subprocess.run("bluetoothctl devices", shell=True, capture_output=True, text=True)
    lines = result.stdout.strip().split("\n")
    devices = []
    for line in lines:
        parts = line.split(" ", 2)  # Split into: 'Device', address, name
        if len(parts) >= 3:
            addr = parts[1]  # Bluetooth MAC address
            name = parts[2].strip()  # Device name
            if name:  # Ignore devices without a name
                devices.append((addr, name))
    print(f"Found {len(devices)} devices with names")
    return devices

def connect_bluetooth_device(device_info):
    """Connect to a Bluetooth device, trust and pair if needed"""
    addr, name = device_info

    # Trust device to allow automatic connection without prompts
    subprocess.run(f"bluetoothctl trust {addr}", shell=True, capture_output=True)
    # Attempt pairing (may require user interaction)
    subprocess.run(f"bluetoothctl pair {addr}", shell=True, capture_output=True)
    # Connect to the device
    connect_result = subprocess.run(f"bluetoothctl connect {addr}", shell=True, capture_output=True, text=True)

    time.sleep(2)  # Allow connection state to settle
    # Check connection status
    info_result = subprocess.run(f"bluetoothctl info {addr}", shell=True, capture_output=True, text=True)

    if "Connected: yes" in info_result.stdout:
        print(f"Successfully connected to {name}")
        return True
    else:
        print(f"Failed to connect to {name}")
        return False

def connect_bluetooth_devices():
    """Scan, display, and allow user to select Bluetooth device to connect using GPIO buttons"""
    devices = scan_bluetooth_devices()
    if not devices:
        # Display no devices found message on screen
        print("No Bluetooth devices found")
        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)
        draw.text((20, 35), "No Devices Found", font=font, fill="red")
        display.display(img)
        time.sleep(2)
        return

    selected_index = 0
    visible_lines = 4  # Number of devices shown at once on display
    max_index = len(devices) - 1

    while True:
        # Draw device selection menu on display
        img = Image.new("RGB", (160, 80), "black")
        draw = ImageDraw.Draw(img)
        title = "Select Device"
        title_width = draw.textlength(title, font=font)
        draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

        # Calculate which devices to show based on current selection for scrolling effect
        scroll_top = max(0, selected_index - visible_lines + 1) if selected_index >= visible_lines else 0
        scroll_bottom = min(scroll_top + visible_lines, len(devices))

        for i in range(scroll_top, scroll_bottom):
            prefix = "> " if i == selected_index else "  "  # Highlight the selected device
            line = prefix + devices[i][1][:15]  # Display device name truncated to 15 characters
            y_pos = 18 + (i - scroll_top) * 14
            draw.text((10, y_pos), line, font=font, fill="red")

        display.display(img)

        # Check GPIO buttons for navigation or selection
        if GPIO.input(12) == GPIO.LOW:
            wait_for_button_release(12)  # Debounce button press
            selected_index = min(max_index, selected_index + 1)  # Move selection down
            
        elif GPIO.input(21) == GPIO.LOW:
            wait_for_button_release(21)
            selected_index = max(0, selected_index - 1)  # Move selection up

        elif GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)
            # Display connecting message
            img = Image.new("RGB", (160, 80), "black")
            draw = ImageDraw.Draw(img)
            draw.text((40, 35), "Connecting...", font=font, fill="red")
            display.display(img)

            device_info = devices[selected_index]
            success = connect_bluetooth_device(device_info)
            if success:
                # Show success message then launch music player interface
                img = Image.new("RGB", (160, 80), "black")
                draw = ImageDraw.Draw(img)
                draw.text((45, 35), "Connected", font=font, fill="red")
                display.display(img)
                time.sleep(2)
                show_music_player()
                return
            else:
                # Show failure message and retry scanning for devices
                img = Image.new("RGB", (160, 80), "black")
                draw = ImageDraw.Draw(img)
                draw.text((25, 35), "Connection Failed", font=font, fill="red")
                display.display(img)
                time.sleep(2)

                devices = scan_bluetooth_devices()  # Retry device scan
                selected_index = 0
                max_index = len(devices) - 1
                if not devices:
                    print("No Bluetooth devices found after retry")
                    return

        time.sleep(0.05)  # Small delay to reduce CPU load

# Note device list needs a 'return' option and scroll wrapping (not yet implemented)

# === Weather Page ===
def show_weather():
    print("Opening weather page.")
    running_weather = True

    # Create initial image with "Fetching weather data" centered text
    img = Image.new("RGB", (160, 80), "black")
    draw = ImageDraw.Draw(img)
    title = "Weather"
    # Center title text horizontally
    draw.text(((160 - draw.textlength(title, font=font)) // 2, 2), title, font=font, fill="red")
    fetching_text = "Fetching weather data..."
    # Center fetching status text horizontally
    draw.text(((160 - draw.textlength(fetching_text, font=font)) // 2, 24), fetching_text, font=font, fill="red")
    display.display(img)

    # Attempt to send weather request via Bluetooth
    if client_sock:
        try:
            client_sock.send("Get Weather")  # Send command string to request weather
        except Exception as e:
            print(f"Failed to send weather request: {e}")
    else:
        print("Bluetooth connection not available.")
        return  # Exit function early if no Bluetooth connection

    previous_weather = ""  # Store last displayed weather string to avoid redundant redraws
    last_update = 0  # Timestamp of last display update

    while running_weather:
        current_time = time.time()

        # Update display at most once per second with new weather data
        if current_time - last_update >= 1:
            with notification_lock:
                current_weather = latest_notification  # Thread-safe fetch of latest weather data

            # Only update display if weather changed and data contains expected delimiter
            if current_weather != previous_weather and "/" in current_weather:
                previous_weather = current_weather

                # Create fresh image for new weather info display
                img = Image.new("RGB", (160, 80), "black")
                draw = ImageDraw.Draw(img)
                draw.text(((160 - draw.textlength(title, font=font)) // 2, 2), title, font=font, fill="red")

                try:
                    # Split weather data into lines using slash as separator
                    lines = current_weather.strip().split("/")
                    # Display up to 3 lines, centered horizontally with vertical spacing of 16 pixels
                    for i, line in enumerate(lines[:3]):
                        draw.text(((160 - draw.textlength(line.strip(), font=font)) // 2, 24 + i * 16), line.strip(), font=font, fill="red")
                except Exception as e:
                    print(f"Failed to parse weather data: {e}")

                display.display(img)

            last_update = current_time

        # Exit loop on button press (GPIO pin 16 LOW means pressed)
        if GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)  # Wait until button released to debounce
            print("Returning to menu.")
            break  # Exit weather display loop

        time.sleep(0.05)  # Sleep briefly to reduce CPU load and allow responsive button polling

    draw_menu()  # Refresh main menu on exit

# === Navigator Page ===
# Direction storage
latest_navigation = ""  # Holds the latest navigation text to be displayed
navigation_lock = threading.Lock()  # Lock to safely synchronize access to latest_navigation

def show_navigator():
    print("Opening navigation page.")
    running_navigator = True
    nav_font = ImageFont.truetype(font_path, size=12)  # Use smaller font for navigation details
    last_update = 0  # Timestamp of last screen update

    while running_navigator:
        current_time = time.time()

        # Update navigation display once per second
        if current_time - last_update >= 1:
            with navigation_lock:
                nav_text = latest_navigation  # Thread-safe read of navigation data

            img = Image.new("RGB", (160, 80), "black")  # Clear display
            draw = ImageDraw.Draw(img)

            title = "Navigator"
            title_width = draw.textlength(title, font=font)
            # Center title horizontally
            draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

            # Use navigation text or fallback if empty
            content = nav_text if nav_text else "Currently no new Google Maps directions."

            # Word wrap logic to split text into lines that fit within 150 pixels width
            words = content.split()
            lines = []
            current_line = ""

            for word in words:
                test_line = current_line + word + " "
                # Check if test_line fits display width, else start new line
                if draw.textlength(test_line, font=nav_font) < 150:
                    current_line = test_line
                else:
                    lines.append(current_line.strip())  # Save current full line
                    current_line = word + " "  # Start new line

            if current_line:
                lines.append(current_line.strip())  # Append leftover line

            # Draw up to 4 lines starting from y=18 with 14 pixel line spacing and 10 px margin left
            for i in range(min(4, len(lines))):
                draw.text((10, 18 + i * 14), lines[i], font=nav_font, fill="red")

            display.display(img)
            last_update = current_time

        # Exit on button press
        if GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)  # Debounce button press
            print("Returning to menu.")
            break

        time.sleep(0.05)  # Small delay to reduce CPU usage and keep UI responsive

    draw_menu()  # Redraw main menu on exit
                
# === Date and Time Page ===
def show_datetime():
    print("Displaying date and time.")
    running_datetime = True
    last_update = 0  # Track last update to limit screen refresh to 1 second

    while running_datetime:
        now = time.localtime()  # Current local time struct (requires accurate system time via Wi-Fi)
        current_time = time.time()  # Time in seconds since epoch

        # Refresh display once per second
        if current_time - last_update >= 1:
            date_str = time.strftime("%d/%m/%Y", now)  # Format date string
            weekday_str = time.strftime("%A", now)  # Full weekday name
            time_str = time.strftime("%H:%M:%S", now)  # Format time string

            img = Image.new("RGB", (160, 80), "black")  # Create blank display buffer
            draw = ImageDraw.Draw(img)

            # Draw centered title and time strings
            title = "Date & Time"
            title_width = draw.textlength(title, font=font)
            draw.text(((160 - title_width) // 2, 2), title, font=font, fill="red")

            date_width = draw.textlength(date_str, font=font)
            draw.text(((160 - date_width) // 2, 22), date_str, font=font, fill="red")

            weekday_width = draw.textlength(weekday_str, font=font)
            draw.text(((160 - weekday_width) // 2, 36), weekday_str, font=font, fill="red")

            time_width = draw.textlength(time_str, font=font)
            draw.text(((160 - time_width) // 2, 50), time_str, font=font, fill="red")

            display.display(img)  # Update screen with new time
            last_update = current_time

        # Exit loop on button press
        if GPIO.input(16) == GPIO.LOW:
            wait_for_button_release(16)  # Debounce
            print("Returning to menu.")
            break

        time.sleep(0.05)  # Sleep to reduce CPU usage

    draw_menu()  # Show main menu after exit


# === Main Loop ===
def main():
    global selected_index, on_main_menu
    on_main_menu = True
    draw_menu()
    last_draw_time = time.time()  # Used to limit how often menu redraws

    while True:
        try:
            now = time.time()

            # Redraw menu every second to keep screen fresh
            if now - last_draw_time >= 1:
                draw_menu()
                last_draw_time = now

            # UP button pressed
            if GPIO.input(21) == GPIO.LOW:
                wait_for_button_release(21)
                selected_index = (selected_index - 1) % len(menu_items)  # Wrap around
                draw_menu()
                last_draw_time = time.time()

            # DOWN button pressed
            elif GPIO.input(12) == GPIO.LOW:
                wait_for_button_release(12)
                selected_index = (selected_index + 1) % len(menu_items)  # Wrap around
                draw_menu()
                last_draw_time = time.time()

            # SELECT button pressed
            elif GPIO.input(16) == GPIO.LOW:
                wait_for_button_release(16)
                selected_item = menu_items[selected_index]
                print(f"'{selected_item}' selected.")
                on_main_menu = False  # Leave main menu state

                # Call matching function based on selected menu item
                if selected_item.lower() == "notifications":
                    show_notifications()
                elif selected_item.lower() == "call":
                    show_call_menu()
                elif selected_item.lower() == "record video":
                    show_take_video_menu()
                elif selected_item.lower() == "take photo":
                    show_take_photo_menu()
                elif selected_item.lower() == "music player":
                    show_music_player()
                elif selected_item.lower() == "weather":
                    show_weather()
                elif selected_item.lower() == "navigator":
                    show_navigator()
                elif selected_item.lower() == "date & time":
                    show_datetime()
                elif selected_item.lower() == "shutdown":
                    print("Shutting down.")
                    close_bluetooth()  # Clean up Bluetooth socket
                    os.system("sudo shutdown now")  # Trigger shutdown
                    break

                on_main_menu = True  # Return to main menu
                draw_menu()
                last_draw_time = time.time()

            time.sleep(0.01)  # Polling delay to reduce CPU usage

        except KeyboardInterrupt:
            print("Interrupted.")  # Graceful exit on Ctrl+C
            break
        except Exception as e:
            print(f"Error: {e}.")  # Handle unexpected errors
            break

    GPIO.cleanup()  # Reset GPIO state on exit


if __name__ == "__main__":
    # Start Bluetooth listener in a background thread
    bt_thread = threading.Thread(target=bluetooth_listener, daemon=True)
    bt_thread.start()

    # Enter main UI loop
    main()
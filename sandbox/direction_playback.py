import csv
import os
import time

import cv2


IMAGE_EXTENSIONS = (".png", ".jpg", ".jpeg", ".bmp", ".tiff")
LEFT_KEYS = {81, 2424832, 65361, ord("a"), ord("A")}
RIGHT_KEYS = {83, 2555904, 65363, ord("d"), ord("D")}
UP_KEYS = {82, 2490368, 65362, ord("w"), ord("W")}

# Hardcoded settingsv
FOLDER_PATH = "./datasets/Organised/20260306-handFlying-1"  # Change this to your image folder
LABELS_PATH = None  # If None, defaults to <FOLDER_PATH>/directions.csv
DELAY_MS = 50
ROTATION_CODE = cv2.ROTATE_90_COUNTERCLOCKWISE


def list_images(folder):
    files = sorted(
        file_name
        for file_name in os.listdir(folder)
        if file_name.lower().endswith(IMAGE_EXTENSIONS)
    )
    return files


def load_directions(file_path):
    directions = {}
    if not os.path.exists(file_path):
        return directions

    with open(file_path, "r", newline="", encoding="utf-8") as csv_file:
        reader = csv.reader(csv_file)
        for row in reader:
            if len(row) < 2:
                continue
            image_name = row[0].strip()
            direction = row[1].strip().lower()
            if image_name == "image_name" and direction == "direction":
                continue
            if image_name and direction:
                directions[image_name] = direction
    return directions


def save_directions(file_path, image_files, directions):
    with open(file_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["image_name", "direction"])
        for image_name in image_files:
            direction = directions.get(image_name)
            if direction:
                writer.writerow([image_name, direction])


def key_to_direction(key_code):
    if key_code in LEFT_KEYS:
        return "left"
    if key_code in RIGHT_KEYS:
        return "right"
    if key_code in UP_KEYS:
        return "up"
    return None


def draw_overlay(frame, image_name, direction, paused, frame_index, total_frames):
    display = frame.copy()
    direction_text = direction if direction else "none"
    state_text = "paused" if paused else "playing"

    cv2.putText(
        display,
        f"Image: {image_name}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display,
        f"Direction: {direction_text}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display,
        f"State: {state_text} | Frame: {frame_index + 1}/{total_frames}",
        (10, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        display,
        "Controls: Left/Right/Up set direction, Space pause/play, Q quit",
        (10, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (220, 220, 220),
        2,
        cv2.LINE_AA,
    )
    return display


def main():
    if not os.path.isdir(FOLDER_PATH):
        print(f"Image folder not found: {FOLDER_PATH}")
        return

    image_files = list_images(FOLDER_PATH)
    if not image_files:
        print("No images found in the folder")
        return

    labels_path = LABELS_PATH if LABELS_PATH else os.path.join(FOLDER_PATH, "directions.csv")
    directions = load_directions(labels_path)

    active_direction = None
    paused = False
    index = 0
    total = len(image_files)
    dirty = False
    last_advance_time = time.monotonic()

    while index < total:
        image_name = image_files[index]
        image_path = os.path.join(FOLDER_PATH, image_name)
        frame = cv2.imread(image_path)

        if frame is None:
            index += 1
            continue

        frame = cv2.rotate(frame, ROTATION_CODE)

        if image_name in directions:
            active_direction = directions[image_name]
        elif active_direction:
            directions[image_name] = active_direction
            dirty = True

        display_frame = draw_overlay(
            frame,
            image_name,
            active_direction,
            paused,
            index,
            total,
        )
        cv2.imshow("Direction Playback", display_frame)

        key = cv2.waitKeyEx(30)

        if key in (ord("q"), ord("Q"), 27):
            break

        if key == ord(" "):
            paused = not paused
            last_advance_time = time.monotonic()

        chosen_direction = key_to_direction(key)
        if chosen_direction is not None:
            active_direction = chosen_direction
            if directions.get(image_name) != chosen_direction:
                directions[image_name] = chosen_direction
                dirty = True
                save_directions(labels_path, image_files, directions)

        now = time.monotonic()
        if not paused and (now - last_advance_time) * 1000.0 >= DELAY_MS:
            if active_direction and directions.get(image_name) != active_direction:
                directions[image_name] = active_direction
                dirty = True
            index += 1
            last_advance_time = now

    if dirty:
        save_directions(labels_path, image_files, directions)

    cv2.destroyAllWindows()
    print(f"Saved direction labels to: {labels_path}")


if __name__ == "__main__":
    main()

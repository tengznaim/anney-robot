import cv2 as cv
import torch
import os
from torchvision import transforms
from PIL import Image
import numpy as np

model = torch.load("model_080622.pth", map_location=torch.device("cpu"))
model.eval()

train_transforms = transforms.Compose([transforms.ToTensor()])
classes = ["mee_goreng", "nasi_goreng", "nasi_lemak", "roti_canai", "satay"]

capture = cv.VideoCapture(0)

# The initial tracker and global variables
tracker = cv.TrackerKCF_create()
tracked_bounding_box = None
label = None

while True:
    isTrue, frame = capture.read()

    # If there is an existing object, update and track the object.
    if tracked_bounding_box is not None:
        (success, box) = tracker.update(frame)

        if success:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(frame, (x, y), (x + w, y + h),
                         (0, 255, 0), 2)
            cv.putText(frame, label, (x, y),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # If the object has disappeared from the frame, remove the previous tracker and create a new one.
        else:
            del tracker
            tracker = cv.TrackerKCF_create()
            tracked_bounding_box = None

    # If there are no current objects, use the model to detect possible food images in the capture.
    else:
        frame_transformed = train_transforms(
            cv.cvtColor(frame, cv.COLOR_BGR2RGB))
        prediction = model([frame_transformed])[0]

        if len(prediction["scores"]) > 0:
            highest_confidence = torch.argmax(prediction["scores"], axis=0)
            label_idx = prediction["labels"][highest_confidence].item()
            label = classes[label_idx]
            box = prediction["boxes"][highest_confidence]

            # Initialise the tracker with the predicted bounding box.
            tracked_bounding_box = [round(i.item()) for i in box]
            tracker.init(frame, tracked_bounding_box)

            print(f"{label} detected. Attempting to track the object.")

            start_point = (round(box[0].item()), round(box[1].item()))
            end_point = (round(box[2].item()), round(box[3].item()))

            cv.rectangle(frame, start_point, end_point, (0, 255, 0), 2)
            cv.putText(frame, label, start_point,
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv.imshow("Video", frame)

    # Press the d key to stop capture
    if cv.waitKey(20) & 0xFF == ord('d'):
        break

capture.release()
cv.destroyAllWindows()

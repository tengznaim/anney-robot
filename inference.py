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

while True:
    isTrue, frame = capture.read()

    frame_transformed = train_transforms(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
    prediction = model([frame_transformed])[0]

    if len(prediction["scores"]) > 0:
        highest_confidence = torch.argmax(prediction["scores"], axis=0)
        label_idx = prediction["labels"][highest_confidence].item()
        pred_class = classes[label_idx]
        box = prediction["boxes"][highest_confidence]

        start_point = (round(box[0].item()), round(box[1].item()))
        end_point = (round(box[2].item()), round(box[3].item()))

        cv.rectangle(frame, start_point, end_point, (0, 255, 0), 2)
        cv.putText(frame, pred_class, start_point,
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv.imshow("Video", frame)

    if cv.waitKey(20) & 0xFF == ord('d'):  # Press d key to break
        break

capture.release()
cv.destroyAllWindows()

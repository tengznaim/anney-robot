import torch
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor

classes = ['nasi_lemak','nasi_goreng','mee_goreng','roti_canai','satay']

# model = torch.load("/home/mustar/anney-test/src/anney_test_pkg/model_080622.pth",map_location=torch.device("cpu"))

# torch.save(model,"/home/mustar/anney-test/src/anney_test_pkg/downgrade_model_080622.pth",_use_new_zipfile_serialization=False)

model = torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn(pretrained=True)

num_classes = len(classes)
# Get the number of input features for the classifier and create a new head with our number of classes.
in_features = model.roi_heads.box_predictor.cls_score.in_features
model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
#! /usr/bin/env python3
import rospy
from torchvision import transforms
import torch
import cv2 as cv
# from std_msgs.msg import String
from anney_melodic_pkg.msg import Order

# Load the model
model = torch.load("/home/mustar/anney-robot-melodic/src/anney_melodic_pkg/model_080622.pth",map_location=torch.device("cpu"))
model.eval()


def create_list(orders,units):
    ''' This function is to create the order list based on its respective quantity '''

    order_temp = []
    for i in range(len(orders)):
        for j in range(units[i]):
            if(orders[i] == "mi_goreng"):
                order_temp.append("mee_goreng")
            elif(orders[i] == "sate"):
                order_temp.append("satay")
            else:
                order_temp.append(orders[i])

    return order_temp


def callback(data):
    rospy.loginfo(f'Table_num: {data.table_num} {data.orders} {data.units}')

    order_temp = create_list(data.orders,data.units) # Create full foods list
    rospy.loginfo(f'This is temp list: {order_temp}')
    train_transforms = transforms.Compose([transforms.ToTensor()])
    classes = ["mee_goreng", "nasi_goreng", "nasi_lemak", "roti_canai", "satay"]
    
    can_remove = True

    capture = cv.VideoCapture(2) # Initialize video capturing object with camera device index as the parameter

    # The initial tracker and global variables
    tracker = cv.TrackerKCF_create()
    tracked_bounding_box = None
    label = None
 

    while (len(order_temp) != 0):
        isTrue, frame = capture.read()

        # If there is an existing object, update and track the object.
        if tracked_bounding_box is not None:
            (success, box) = tracker.update(frame)

            if success:
                if(can_remove and label in order_temp):
                    rospy.loginfo(f'{label} for table {data.table_num} is done!')
                    order_temp.remove(label)
                    rospy.loginfo(f'This is temp list: {order_temp}')
                    can_remove = False

                (x, y, w, h) = [int(v) for v in box]
                cv.rectangle(frame, (x, y), (x + w, y + h),(0, 255, 0), 2)
                cv.putText(frame, label, (x, y),cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # If the object has disappeared from the frame, remove the previous tracker and create a new one.
            else:
                del tracker
                tracker = cv.TrackerKCF_create()
                tracked_bounding_box = None
                can_remove = True

        # If there are no current objects, use the model to detect possible food images in the capture.
        else:
            frame_transformed = train_transforms(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
            prediction = model([frame_transformed])[0] # The model detects from the frame

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

            
        if cv.waitKey(20) & 0xFF == ord("d"):
            break
    
    rospy.loginfo(f'All orders are done, can start to serve to table {data.table_num} now!')
    capture.release()
    cv.destroyAllWindows()

    


def listener():
    # Initialize subscriber node and let it subscribe to a topic
    rospy.init_node("Subscriber_Node", anonymous=True)
    rospy.Subscriber("order_topic", Order, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
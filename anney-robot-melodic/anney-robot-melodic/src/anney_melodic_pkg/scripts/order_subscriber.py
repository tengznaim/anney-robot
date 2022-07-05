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

    order_temp = create_list(data.orders,data.units)
    train_transforms = transforms.Compose([transforms.ToTensor()])
    classes = ["mee_goreng", "nasi_goreng", "nasi_lemak", "roti_canai", "satay"]

    capture = cv.VideoCapture(2) # Initialize video capturing object with camera device index as the parameter
 

    while (len(order_temp) != 0):
        isTrue, frame = capture.read()

        frame_transformed = train_transforms(cv.cvtColor(frame, cv.COLOR_BGR2RGB))
        prediction = model([frame_transformed])[0] # The model detects from the frame

        # If there is detection, draw bounding box + food class text overlay
        if len(prediction["scores"]) > 0:
            highest_confidence = torch.argmax(prediction["scores"], axis=0)
            label_idx = prediction["labels"][highest_confidence].item()
            pred_class = classes[label_idx]
            box = prediction["boxes"][highest_confidence]

            start_point = (round(box[0].item()), round(box[1].item()))
            end_point = (round(box[2].item()), round(box[3].item()))

            cv.rectangle(frame, start_point, end_point, (0, 255, 0), 2)
            cv.putText(frame, pred_class, start_point,cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Remove the food in the list if detected (consider it's ready for the customer)
            if(pred_class in order_temp):
                rospy.loginfo(f'{pred_class} is done for table {data.table_num}')
                order_temp.remove(pred_class)
        
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
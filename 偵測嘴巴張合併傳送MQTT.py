import dlib
import cv2
import imutils
from time import sleep
import time
import paho.mqtt.client as mqtt

#MQTT設置
#這部分是使電腦執行，因此不用連網路
client = mqtt.Client()
client.connect("140.127.218.172", 1883)

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor( 'shape_predictor_68_face_landmarks.dat')
while(cap.isOpened()):
    ret, frame = cap.read()
    face_rects, scores, idx = detector.run(frame, 1)
    for i, d in enumerate(face_rects):
        x1 = d.left()
        y1 = d.top()
        x2 = d.right()
        y2 = d.bottom()
        text = " %2.2f ( %d )" % (scores[i],idx[i])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (127, 255, 0), 4, cv2. LINE_AA)
        cv2.putText(frame, text, (x1, y1), cv2. FONT_HERSHEY_DUPLEX,0.7, (127, 255, 0), 1, cv2. LINE_AA)

        landmarks_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        shape = predictor(landmarks_frame, d)
        for i in range(68):
            if i in [48,51,54,57]:
                cv2.putText(frame, str(i),(shape.part(i).x,shape.part(i).y),cv2. FONT_HERSHEY_PLAIN, 0.5,(255, 255, 187), 1)
                top =  shape.part(51).y
                bottom =  shape.part(57).y
                left = shape.part(48).x
                right =  shape.part(54).x

                height = bottom - top
                width = right - left
                r = height / (width + 0.01) #取得嘴巴高度及寬度的比例
                r = r*100 #將該比例調整成伺服馬達可接受的數值範圍
                client.publish("A1083357_Test", r) #設定topic和內容廣播到MQTT上
                print(r)
                #time.sleep(0.5)
        cv2.imshow( "Face Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
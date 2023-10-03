from time import sleep
import time
import cv2
import mediapipe as mp
import paho.mqtt.client as mqtt

#MQTT設置
#這部分是使電腦執行，因此不用連網路
client = mqtt.Client()
client.connect("140.127.218.172", 1883)

count = 0 # 設置一個數來代替time sleep，防止相機偵測卡頓

mp_drawing = mp.solutions.drawing_utils
mp_face_mesh = mp.solutions.face_mesh
##----- Camera ---
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
cap = cv2.VideoCapture(0)
with mp_face_mesh.FaceMesh(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as face_mesh:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # Flip the image horizontally for a later selfie-view display, and convert
    # the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    shape = image.shape 
    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    results = face_mesh.process(image)

    # Draw the face mesh annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    xm={}
    ym={}
    if results.multi_face_landmarks:
      for face_landmarks in results.multi_face_landmarks:
        for i in range(4):
          
          mark=[12,14,57,287]
          #print(i,mark[i])
          xm[i]=int(face_landmarks.landmark[mark[i]].x*shape[1])
          ym[i]=int(face_landmarks.landmark[mark[i]].y*shape[0])
          cv2.circle(image, (xm[i], ym[i]), radius=4, color=(225, 0, 0), thickness=1)
        cv2.line(image,(xm[0],ym[0]),(xm[1],ym[1]),color=(0, 255, 0), thickness=1)
        cv2.line(image,(xm[2],ym[2]),(xm[3],ym[3]),color=(0, 0, 255), thickness=1)
        DMWidth=((xm[2] - xm[3]) ** 2 + (ym[2] - ym[3]) ** 2) ** 0.5
        DMHeight=((xm[0] - xm[1]) ** 2 + (ym[0] - ym[1]) ** 2) ** 0.5
        #print("%.2f\t%.2f\t%.2f" %(DMWidth,DMHeight,DMHeight/(DMWidth+1)))
        r = (DMHeight/(DMWidth+1))*300 # 取得嘴巴高度及寬度的比例，將該比例調整成伺服馬達可接受的數值範圍
        if (count == 15):
            client.publish("A1083357_Test", r) #設定topic和內容廣播到MQTT上
            count = 0
            print(r)
        count += 1
        #print(r)
        #print(face_landmarks.landmark[0].x)
        #mp_drawing.draw_landmarks(
        #    image=image,
        #    landmark_list=face_landmarks,
        #    #connections=mp_face_mesh.FACE_CONNECTIONS,
        #    landmark_drawing_spec=drawing_spec,
        #    connection_drawing_spec=drawing_spec)
    cv2.imshow('MediaPipe FaceMesh', image)
    if cv2.waitKey(5) & 0xFF == 27:
      break
cap.release()
cv2.destroyAllWindows()
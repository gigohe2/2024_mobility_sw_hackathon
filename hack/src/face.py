import cv2
import rospy
from std_msgs.msg import Int32



class Facedetector():
    def __init__(self):
        self.classifier = cv2.CascadeClassifier('/home/jun/catkin_ws/src/hack/src/haarcascade_frontalface_alt2.xml')
        self.face_pub = rospy.Publisher("/face_y", Int32, queue_size=1)
        rospy.init_node("face")


    def main(self):

        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            
    
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # 얼굴을 인식합니다.
            faces = self.classifier.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            if len(faces) == 1:
                face_y = faces[:,1]+int(faces[:,3]/2)
                self.face_pub.publish(int(face_y))
            # 인식된 얼굴에 대해 사각형을 그리고, 좌표를 출력합니다.
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # print("Face found at - X: " + str(x) + " Y: " + str(y))

            # 결과를 화면에 표시합니다.
            cv2.imshow('Video', frame)
            cv2.waitKey(1)
            

if __name__ == "__main__":

    face = Facedetector()
    
    face.main()

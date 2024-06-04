import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QPushButton, QLabel, QComboBox, QSpinBox, QListWidget, QMessageBox
from queue import Queue
import rospy
from std_msgs.msg import String
from klick.srv import OrderPose, OrderPoseRequest, OrderPoseResponse

class KioskGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("키오스크 주문 시스템")
        self.setGeometry(100, 100, 800, 600)

        self.delivery_queue = Queue()
        self.takeout_queue = Queue()

        self.order_number = 1

        self.initUI()
        self.initROS()

    def initUI(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()

        self.delivery_takeout_label = QLabel("테이크아웃 또는 배달을 선택하세요:")
        layout.addWidget(self.delivery_takeout_label)

        self.delivery_takeout_combo = QComboBox()
        self.delivery_takeout_combo.addItems(["테이크아웃", "배달"])
        layout.addWidget(self.delivery_takeout_combo)

        self.menu_label = QLabel("메뉴를 선택하세요:")
        layout.addWidget(self.menu_label)

        self.menu_combo = QComboBox()
        self.menu_combo.addItems(["아메리카노", "카페라떼"])
        layout.addWidget(self.menu_combo)

        self.option_label = QLabel("옵션을 선택하세요:")
        layout.addWidget(self.option_label)

        self.option_hot_ice_combo = QComboBox()
        self.option_hot_ice_combo.addItems(["아이스", "핫"])
        layout.addWidget(self.option_hot_ice_combo)

        self.option_syrup_combo = QComboBox()
        self.option_syrup_combo.addItems(["시럽(X)", "시럽(O)"])
        layout.addWidget(self.option_syrup_combo)

        self.count_label = QLabel("잔수를 선택하세요:")
        layout.addWidget(self.count_label)

        self.count_spinbox = QSpinBox()
        self.count_spinbox.setRange(1, 10)
        layout.addWidget(self.count_spinbox)

        self.order_button = QPushButton("주문하기")
        self.order_button.clicked.connect(self.place_order)
        layout.addWidget(self.order_button)

        self.order_list = QListWidget()
        layout.addWidget(self.order_list)

        central_widget.setLayout(layout)

    def initROS(self):
        rospy.init_node('kiosk_gui_node', anonymous=True)
        self.order_pub = rospy.Publisher('order_topic', String, queue_size=10)
        rospy.wait_for_service('order_pose')
        self.order_service = rospy.ServiceProxy('order_pose', OrderPose)

    def place_order(self):
        order_type = self.delivery_takeout_combo.currentText()
        menu = self.menu_combo.currentText()
        option_hot_ice = self.option_hot_ice_combo.currentText()
        option_syrup = self.option_syrup_combo.currentText()
        count = self.count_spinbox.value()

        order_details = f"주문번호: {self.order_number}, 유형: {order_type}, 메뉴: {menu}, 옵션: {option_hot_ice}, {option_syrup}, 잔수: {count}"

        if order_type == "배달":
            self.delivery_queue.put(order_details)
        else:
            self.takeout_queue.put(order_details)

        self.order_list.addItem(order_details)
        self.order_number += 1

        # 주문 완료 메시지와 ROS publish
        QMessageBox.information(self, "주문 완료", "주문이 완료되었습니다!")
        self.order_pub.publish(order_details)

        # ROS 서비스 클라이언트를 통해 주문 처리
        self.send_order_to_robot(order_type, menu, option_hot_ice, option_syrup, count)

    def send_order_to_robot(self, order_type, menu, option_hot_ice, option_syrup, count):
        request = OrderPoseRequest()
        request.order_number = self.order_number - 1
        request.order_type = order_type
        request.drink_type = "americano" if menu == "아메리카노" else "latte"
        request.temperature = "ice" if option_hot_ice == "아이스" else "hot"
        request.syrup = option_syrup == "시럽(O)"
        request.count = count

        try:
            response = self.order_service(request)
            if response.success:
                rospy.loginfo(f"Order processed successfully in {response.execution_time} seconds.")
            else:
                rospy.logwarn("Order processing failed.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


    def get_next_order(self, order_type):
        if order_type == "배달" and not self.delivery_queue.empty():
            return self.delivery_queue.get()
        elif order_type == "테이크아웃" and not self.takeout_queue.empty():
            return self.takeout_queue.get()
        return None

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = KioskGUI()
    window.show()
    sys.exit(app.exec_())

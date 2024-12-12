import rclpy
import rclpy.duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer her 1 saniyede bir tf verisini kontrol eder
        self.timer = self.create_timer(0.25, self.lookup_transform)
        self.gt_x = []
        self.gt_y = []
        self.est_x = []
        self.est_y = []
        self.number_of_data_points = 300

    def zoom(self, img, zoom_factor=2):
        return cv2.resize(img, None, fx=zoom_factor, fy=zoom_factor)

    def lookup_transform(self):
        try:
            transform_gt: TransformStamped = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                rclpy.time.Time(seconds=0)
            )

            transform_gt_map_to_odom: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(seconds=0)
            )

            print("gt: x=",transform_gt.transform.translation.x+transform_gt_map_to_odom.transform.translation.x,
                  "y=",transform_gt.transform.translation.y+transform_gt_map_to_odom.transform.translation.y)
            
            self.gt_x.append(transform_gt.transform.translation.x+transform_gt_map_to_odom.transform.translation.x)
            self.gt_y.append(transform_gt.transform.translation.y+transform_gt_map_to_odom.transform.translation.y)

            transform_est: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'odom_cam',
                rclpy.time.Time(seconds=0)
            )

            print("est: x=",transform_est.transform.translation.x,
                  "y=",transform_est.transform.translation.y)
            
            self.est_x.append(transform_est.transform.translation.x)
            self.est_y.append(transform_est.transform.translation.y)
            
            if len(self.est_x)==self.number_of_data_points+10:
                datas = {'gt_x':self.gt_x[-self.number_of_data_points:],
                         'gt_y':self.gt_y[-self.number_of_data_points:],
                         'est_x':self.est_x[-self.number_of_data_points:],
                         'est_y':self.est_y[-self.number_of_data_points:]}
                
                df = pd.DataFrame(datas)

                df = df.assign(diff_x=df['est_x'] - df['gt_x']) 
                df = df.assign(diff_y=df['est_y'] - df['gt_y'])
                df = df.assign(dist=np.sqrt(df['diff_x']**2 + df['diff_y']**2))
                df.to_csv("/home/oozdemir/Desktop/eval.csv")

                """total_gt_x_distance=np.sum(np.abs(np.array(df['gt_x'])))
                total_gt_y_distance=np.sum(np.abs(np.array(df['gt_y'])))
                total_gt_distance=np.sqrt(total_gt_x_distance**2+total_gt_y_distance**2)

                total_est_x_distance=np.sum(np.abs(np.array(df['est_x'])))
                total_est_y_distance=np.sum(np.abs(np.array(df['est_y'])))
                total_est_distance=np.sqrt(total_est_x_distance**2+total_est_y_distance**2)

                avg_error_x = np.mean(np.array(df['diff_x']))
                avg_error_y = np.mean(np.array(df['diff_y']))
                avg_error = np.sqrt(avg_error_x**2+avg_error_y**2)"""

                rmse = np.sqrt(np.sum(df['diff_x']**2)/self.number_of_data_points)

                error = ("RMSE: %.2f meters"%(rmse))

                paths = np.zeros((1040,1040,3),np.uint8)
                path_pts_gt=np.column_stack((np.array(df['gt_x']),np.array(df['gt_y'])))
                path_pts_est=np.column_stack((np.array(df['est_x']),np.array(df['est_y'])))

                for i in range(0,self.number_of_data_points):
                    paths[1040-int((path_pts_gt[i][0]+13)*40),
                          1040-int((path_pts_gt[i][1]+13)*40)]=(255,0,0)
                    paths[1040-int((path_pts_est[i][0]+13)*40),
                          1040-int((path_pts_est[i][1]+13)*40)]=(0,255,0)

                cropped = paths[260:780, 260:780]
                paths = self.zoom(cropped)

                cv2.putText(paths,error,(0,970),cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,color=(255,255,255),thickness=2)
                cv2.putText(paths,'Ground Truth Path',(0,995),cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,color=(255,0,0),thickness=2)
                cv2.putText(paths,'Estimated Path',(0,1020),cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.8,color=(0,255,0),thickness=2)
                
                cv2.imwrite('/home/oozdemir/Desktop/results.png',paths)

                x_axis = np.arange(0,self.number_of_data_points) / 4

                plt.figure()
                plt.xlabel('Time(s)')
                plt.ylabel('Error in Distance(m)')
                plt.plot(x_axis, df['dist'])
                plt.show()

                sys.exit()

        except LookupException as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

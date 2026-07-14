# 使用Python进行转换的示例代码
import tf.transformations as tf

quaternion = (-0.5,0.5, -0.5, 0.5)
euler = tf.euler_from_quaternion(quaternion)
print(f"Roll: {euler[0]}, Pitch: {euler[1]}, Yaw: {euler[2]}")
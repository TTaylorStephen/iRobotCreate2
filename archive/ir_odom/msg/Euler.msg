# this represents the pitch, roll, and yaw values as determined through imu values directly and threough fltering. Can get # pitch reliably from complimentary filter using acc and gyro values. Need madgwick filter to determine yaw anmd roll
# reliably due to drift

float64 aPitch
float64 aRoll

float64 gPitch
float64 gRoll
float64 gYaw

float64 mYaw

float64 Pitch
float64 Roll
float64 Yaw



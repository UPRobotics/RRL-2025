import depthai as dai
import cv2

pipeline = dai.Pipeline()
rgb = pipeline.create(dai.node.ColorCamera)
rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
rgb.setPreviewSize(640, 400)

devices = dai.Device.getAllConnectedDevices()
if not devices:
    print("No DepthAI devices found")
    exit()
device_info = devices[0]

with dai.Device(device_info) as device:
    device.startPipeline(pipeline)
    queue = rgb.preview.createOutputQueue(maxSize=4, blocking=False)
    while True:
        frame = queue.tryGet()
        if frame is not None:
            cv2.imshow("RGB", frame.getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
cv2.destroyAllWindows()
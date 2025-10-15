# import pyrealsense2 as rs
# import numpy as np

# class py_Realsense():
#     def __init__(self):
#         self.pipeline = rs.pipeline()
#         self.pc = rs.pointcloud()
#         self.config = rs.config()
#         self.align_to = rs.stream.color
#         self.align = rs.align(self.align_to)

#         context = rs.context()
#         device = context.devices[0]
#         serial = device.get_info(rs.camera_info.serial_number)
#         print(f"Connected to: {serial}")

#         self.config.enable_device(serial)
#         self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

#     def capture(self):
#         self.pipeline.start(self.config)
#         frames = self.pipeline.wait_for_frames()
#         aligned = self.align.process(frames)

#         depth = aligned.get_depth_frame()
#         color = aligned.get_color_frame()
#         color_img = np.asanyarray(color.get_data())

#         self.pc.map_to(color)
#         points = self.pc.calculate(depth)
#         vertices = np.array(points.get_vertices())
#         colors = color_img.reshape(-1, 3)

#         self.pipeline.stop()
#         return vertices, colors, depth

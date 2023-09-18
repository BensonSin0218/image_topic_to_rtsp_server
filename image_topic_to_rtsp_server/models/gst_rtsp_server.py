import gi
gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer

class ImageFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, name: str, image_profile: tuple, get_image: callable, \
        fps: int = 30, bitrate: int = 1024, **properties) -> None:
        super().__init__(**properties)

        self.name: str = name
        self.image_profile: tuple = image_profile
        self.get_image: callable = get_image

        self.number_frames: int = 0
        self.fps = fps
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds

        self.launch_string: str = " ! ".join([
            f"appsrc name={self.name} is-live=true block=true format=GST_FORMAT_TIME "\
                f"caps=video/x-raw,format=RGB,height={self.image_profile[0]},width={self.image_profile[1]}",
            "videoconvert ! video/x-raw",
            f"x264enc bitrate={bitrate} speed-preset=ultrafast quantizer=20 tune=zerolatency",
            "h264parse ! rtph264pay config-interval=1 name=pay0 pt=96",
        ])

    def do_create_element(self, _) -> any:
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, rtsp_media) -> None:
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name(self.name)
        appsrc.connect("need-data", self.on_need_data)

    def on_need_data(self, src, _) -> None:
        image = self.get_image(self.name)
        if image is None:
            return None

        data = image.tostring()
        buffer = Gst.Buffer.new_allocate(None, len(data), None)
        buffer.fill(0, data)
        buffer.duration = self.duration
        timestamp = self.number_frames * self.duration
        buffer.pts = buffer.dts = int(timestamp)
        buffer.offset = timestamp

        self.number_frames += 1
        push_buffer_response = src.emit("push-buffer", buffer)

        if push_buffer_response != Gst.FlowReturn.OK:
            print(push_buffer_response)

class ImageToGstRTSPServer(GstRtspServer.RTSPServer):
    def __init__(self, incoming_address: str, incoming_port: int, factory_configs: dict, **properties):
        super().__init__(**properties)

        self.set_address(incoming_address)
        self.set_service(str(incoming_port))

        for source, configs in factory_configs.items():
            factory = ImageFactory(source, configs["profile"], configs["get_image"])
            factory.set_shared(True)
            self.get_mount_points().add_factory(f"/streaming/{source}", factory)

        self.attach(None)

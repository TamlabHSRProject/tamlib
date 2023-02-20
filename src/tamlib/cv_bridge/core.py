import struct

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge as _CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class CvBridge:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        self.bridge = _CvBridge()

    def imgmsg_to_cv2(self, img_msg: Image, encoding="passthrough") -> np.ndarray:
        """Imageメッセージをcv2形式の画像に変換する

        Args:
            img_msg (Image): Imageメッセージ．
            encoding (str, optional): エンコーディング. Defaults to "passthrough".

        Returns:
            np.ndarray: cv2形式の画像．
        """
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, encoding)
            return cv_img
        except CvBridgeError as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: CvBridge FAILURE.")
            return None

    def imgmsg_to_depth(self, img_msg: Image) -> np.ndarray:
        """Imageメッセージをcv2形式のDepth画像に変換する

        Args:
            img_msg (Image): Imageメッセージ．

        Returns:
            np.ndarray: cv2形式のDepth画像．
        """
        cv_img = self.imgmsg_to_cv2(img_msg, "32FC1")
        return cv_img.astype(np.uint16)

    def compressed_imgmsg_to_cv2(
        self, img_msg: CompressedImage, encoding="passthrough"
    ) -> np.ndarray:
        """CompressedImageメッセージをcv2形式の画像に変換する

        Args:
            img_msg (CompressedImage): CompressedImageメッセージ
            encoding (str, optional): エンコーディング. Defaults to "passthrough".

        Returns:
            np.ndarray: cv2形式の画像．
        """
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg, encoding)
            return cv_img
        except CvBridgeError as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: CvBridge FAILURE.")
            return None

    def compressed_imgmsg_to_depth(
        self, img_msg: CompressedImage, header_size=12
    ) -> np.ndarray:
        """CompressedImageメッセージをcv2形式のDepth画像に変換する

        Args:
            img_msg (CompressedImage): CompressedImageメッセージ
            header_size (int): ヘッダーサイズ．Defaults to 12.

        Raises:peg.decode_header(in_file.read())
            Exception: メッセージの型が'compressedDepth'でない場合
            Exception: メッセージのヘッダーサイズが定義と異なる場合
            Exception: '16UC1'または'32FC1'以外でエンコードされているメッセージの場合

        Returns:
            np.ndarray: cv2形式のDepth画像．
        """
        try:
            encode, _type = img_msg.format.split(";")
            encode = encode.strip()
            _type = _type.strip()
            if "compressedDepth" not in _type:
                raise Exception("Compression type is not 'compressedDepth'.")
            raw_data = img_msg.data[header_size:]

            depth_img_raw = cv2.imdecode(
                np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED
            )
            if depth_img_raw is None:
                raise Exception("Could not decode compressed depth image.")

            if encode == "16UC1":
                return depth_img_raw
            elif encode == "32FC1":
                raw_header = img_msg.data[:header_size]
                _, depth_quant_a, depth_quant_b = struct.unpack("iff", raw_header)
                depth_img_scaled = depth_quant_a / (
                    depth_img_raw.astype(np.float32) - depth_quant_b
                )
                depth_img_scaled[depth_img_raw == 0] = 0
                depth_img_mm = (depth_img_scaled * 1000).astype(np.uint16)
                return depth_img_mm
            else:
                raise Exception("Decoding of '" + encode + "' is not implemented.")
        except Exception as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: CvBridge FAILURE.")
            return None

    def cv2_to_imgmsg(self, cv_img: np.ndarray, encoding="passthrough") -> Image:
        """cv2形式の画像をImageメッセージに変換する

        Args:
            cv_img (np.ndarray): cv2形式の画像
            encoding (str, optional): エンコーディング. Defaults to "passthrough".

        Returns:
            Image: Imageメッセージ
        """
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding)
            return img_msg
        except CvBridgeError as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: CvBridge FAILURE")
            return None

    def cv2_to_compressed_imgmsg(
        self, cv_img: np.ndarray, format="jpg"
    ) -> CompressedImage:
        """cv2形式の画像をCompressedImageメッセージに変換する

        Args:
            cv_img (np.ndarray): cv2形式の画像
            format (str, optional): 圧縮フォーマット. Defaults to "jpg".
                フォーマット: bmp, dib, jpeg, jpg, jpe, jp2, png, pbm, pgm, ppm, sr, ras, tiff, tif

        Returns:
            CompressedImage: CompressedImageメッセージ
        """
        try:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img, format)
            return img_msg
        except CvBridgeError as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: CvBridge FAILURE")
            return None

    def depth_to_compressed_imgmsg(
        self, cv_img: np.ndarray, format="png"
    ) -> CompressedImage:
        """cv2形式の画像をCompressedImageメッセージに変換する
            16UC1のみ対応

        Args:
            cv_img (np.ndarray): cv2形式のDepth画像
            format (str, optional): 圧縮フォーマット. Defaults to "png".
                フォーマット: bmp, dib, jpeg, jpg, jpe, jp2, png, pbm, pgm, ppm, sr, ras, tiff, tif

        Returns:
            CompressedImage: CompressedImageメッセージ
        """
        try:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(cv_img, format)
            img_msg.format = f"16UC1; compressedDepth {format}"
            return img_msg

        except CvBridgeError as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: CvBridge FAILURE")
            return None

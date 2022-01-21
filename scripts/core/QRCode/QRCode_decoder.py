#!/usr/bin/env python3
from pyzbar.pyzbar import decode
import numpy as np
import typing
import PIL.Image
import cv2


class QRCodeDecoder:
    @staticmethod
    def decode(img: typing.Union[np.ndarray, str]) -> typing.List:
        if isinstance(img, np.ndarray) or isinstance(img, PIL.Image.Image):
            return decode(img)
        if isinstance(img, str):
            return decode(cv2.imread(img))

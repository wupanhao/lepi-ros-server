from pyzbar import pyzbar
from camera_utils import putText3


class BarcodeScanner:
    def __init__(self):
        self.barcodes = []
        self.barcode = ['', 0, 0, 0, 0]

    def detectBarcode(self, frame):
        self.barcodes = pyzbar.decode(frame)
        self.barcode = self.getBarcodeData()
        return self.drawLabel(frame, self.barcodes)

    def getBarcodeData(self, i=0):
        if len(self.barcodes) > i:
            barcode = self.barcodes[i]
            text = barcode.data.decode("utf-8")
            (x, y, w, h) = barcode.rect
            return [text, int(x+w/2), int(y+h/2), w, h]
        else:
            return ['', 0, 0, 0, 0]

    def detectedBarcode(self, pattern):
        for i, barcode in enumerate(self.barcodes):
            text = barcode.data.decode("utf-8")
            if text.find(pattern) >= 0:
                self.barcode = self.getBarcodeData(i)
                # print(self.barcodes,self.barcode)
                return True
        self.barcode = ['', 0, 0, 0, 0]
        return False

    def drawLabel(self, frame, barcodes):
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # barcodeData = barcode.data.decode("utf-8")
            # barcodeType = barcode.type

            text = barcode.data.decode("utf-8")
            # print(text,text.decode("utf-8"))
            # text = "{} ({})".format(barcodeData, barcodeType)
            # cv2.putText(frame, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            frame = putText3(frame, text, (x, y - 10), (0, 0, 255))
        return frame

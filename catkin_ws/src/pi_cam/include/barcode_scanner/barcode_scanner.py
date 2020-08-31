from pyzbar import pyzbar

class BarcodeScanner:
    def __init__(self):
        self.barcodes = []
        self.barcode = ['',0,0,0,0]

    def detectBarcode(self,frame):
        self.barcodes = pyzbar.decode(frame)
        self.barcode = self.getBarcodeData()

    def getBarcodeData(self,i=0):
        if len(self.barcodes) > i:
            barcode = self.barcodes[i]
            text = barcode.data.decode("utf-8")
            (x, y, w, h) = barcode.rect
            return [text,int(x+w/2),int(y+h/2),w,h]
        else:
            return ['',0,0,0,0]

    def detectedBarcode(self,pattern):
        for i,barcode in enumerate(self.barcodes):
            text = barcode.data.decode("utf-8")
            if text.find(pattern)>=0:
                self.barcode = self.getBarcodeData(i)
                # print(self.barcodes,self.barcode)
                return True
        self.barcode = ['',0,0,0,0]
        return False
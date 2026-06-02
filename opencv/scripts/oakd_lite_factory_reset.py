import depthai as dai

device = dai.Device()
device.factoryReset()
print("Factory reset sent")

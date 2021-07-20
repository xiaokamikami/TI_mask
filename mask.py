
import sensor, image, lcd, time
import KPU as kpu

color_R = (255, 0, 0)
color_G = (0, 255, 0)
color_B = (0, 0, 255)


class_IDs = ['no_mask', 'mask']

umask = GPIO(11,GPIO.OUT_PP , GPIO.PULL_UP,0)
mask = GPIO(10, GPIO.OUT_PP , GPIO.PULL_UP, 0)
mask.value(0)
umask.value(0)
def drawConfidenceText(image, rol, classid, value):
    text = ""
    _confidence = int(value * 100)

    if classid == 1:
        text = 'mask: ' + str(_confidence) + '%'
    else:
        text = 'no_mask: ' + str(_confidence) + '%'

    image.draw_string(rol[0], rol[1], text, color=color_R, scale=2.5)


class girlfirend:

lcd.init()
sensor.reset(dual_buff=True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(0)
sensor.set_hmirror(1)    #设置摄像头镜像
sensor.set_vflip(1)      #设置摄像头翻转
sensor.run(1)


task = kpu.load(0x300000)


anchor = (0.1606, 0.3562, 0.4712, 0.9568, 0.9877, 1.9108, 1.8761, 3.5310, 3.4423, 5.6823)
_ = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)
img_lcd = image.Image()


while (True):

    img = sensor.snapshot()
    code = kpu.run_yolo2(task, img)
    if code:
        totalRes = len(code)

        for item in code:
            confidence = float(item.value())
            itemROL = item.rect()
            classID = int(item.classid())

            if classID == 1 and confidence > 0.65:
                _ = img.draw_rectangle(itemROL, color_G, tickness=5)
                if totalRes == 1:
                    mask.value(1)
                    umask.value(0)
                    drawConfidenceText(img, (0, 0), 1, confidence)
            else:
                _ = img.draw_rectangle(itemROL, color=color_R, tickness=5)
                if totalRes == 1:
                    umask.value(1)
                    mask.value(0)
                    drawConfidenceText(img, (0, 0), 0, confidence)

    _ = lcd.display(img)
    else:
        umask.value(0)
        mask.value(0)


_ = kpu.deinit(task)

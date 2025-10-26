# 添加上层目录到sys
import os, sys
parent_path = os.path.abspath(os.path.join(__file__, *(['..'] * 2)))
sys.path.insert(0, parent_path)
from camera import Camera
import cv2, time
import numpy as np
from paddle_jetson import MotHuman, YoloeInfer, HummanAtrr, OCRReco, LaneInfer

def yoloe_test():
    cap = Camera(2)
    # infer = YoloeInfer("front_model", "trt_fp32")
    # infer = YoloeInfer("front_model")
    # infer = YoloeInfer("ppyoloe_crn_s_400e_coco_wbt")
    # infer = YoloeInfer("front_model2", "trt_fp16")
    # infer = YoloeInfer("front_model2", "trt_fp32")
    # infer = YoloeInfer("front_model", "trt_fp16")
    # infer = YoloeInfer("mot_ppyoloe_s_36e_pipeline")
    # infer = YoloeInfer("mot_ppyoloe_s_36e_pipeline")
    infer = YoloeInfer("task_model3")
    # infer = YoloeInfer("ppyoloe_plus_crn_t_auxhead_448_300e_coco")
    print("start ok")
    time_l = time.time()
    while True:
        img = cap.read()
        response = infer.predict(img)
        print(response)
        for ret in response:
            # cv画出方框
            label = ret.label_name
            bbox = ret.bbox
            cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
            cv2.putText(img, label, (bbox[0], bbox[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        fps = int(1 / (time.time() - time_l))
        time_l = time.time()
        print("fps:", fps)
        # cv2.putText(img, "fps:{}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("img", img)
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
    cap.close()
    cv2.destroyAllWindows()

def hum_mot_test():
    cap = Camera(2)
    cap.start_back_thread()
    mot_hum = MotHuman()
    while True:
        time_l = time.time()
        img = cap.read()
        res = mot_hum.predict(img, visualize=True)
        fps = int(1 / (time.time() - time_l))
        # time_l = time.time()

        print("fps:", fps)
        # cv2.putText(img, "fps:{}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print(res)
        cv2.imshow("img", img)
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
    cv2.destroyAllWindows()
    cap.close()

def OCR_test():
    image = cv2.imread("12.jpg")
    ocr = OCRReco()
    res = ocr.predict(image)
    # for bbox in res:
    #     bbox = bbox.astype(np.int32)
    #     print(bbox)
    #     cv2.line(image, bbox[0], bbox[1], (0, 0, 255), 2)
    #     cv2.line(image, bbox[1], bbox[2], (0, 0, 255), 2)
    #     cv2.line(image, bbox[2], bbox[3], (0, 0, 255), 2)
    #     cv2.line(image, bbox[3], bbox[0], (0, 0, 255), 2)
    #     # cv2.putText(im, bbox[4], (bbox[0][0], bbox[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    # cv2.imwrite("res.jpg", image)
    print(res)


def attr_test():
    im = cv2.imread("h5.jpg")
    infer = HummanAtrr()
    response = infer(im)
    
    print(response)

def cam_test():
    cap = Camera(2)
    cap.start_back_thread()
    # ocr = OCRReco()
    fps = 0
    start_time = time.time()
    while True:
        img = cap.read()
        # res = ocr.predict(img)
        # print(res)
        fps = int(1 / (time.time() - start_time))
        start_time = time.time()
        # print("fps:", fps)
        # cv2.putText(img, "fps:{}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("img", img)
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
    cap.close()

def infer_test():
    im = cv2.imread("1.png")
    # infer = YoloInfer()
    infer = LaneInfer()
    response = infer(im)
    print(response)

def cam_infer_test():
    cap = cv2.VideoCapture(0)
    infer = LaneInfer()
    time_start = time.time()
    while True:
        ret, img = cap.read()
        if ret:
            response = infer(img)
            print(response)
            fps = 1/(time.time()-time_start)
            print("fps:", fps)
            time_start = time.time()
            # cv2.putText(img, "fps:{}".format(fps), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # cv2.imshow("img", img)
            key = cv2.waitKey(10)
            if key == ord('q'):
                break

if __name__ == "__main__":
    yoloe_test()
    # hum_mot_test()
    # attr_test()
    # infer_test()
    # cam_test()
    # OCR_test()
    # cam_infer_test()
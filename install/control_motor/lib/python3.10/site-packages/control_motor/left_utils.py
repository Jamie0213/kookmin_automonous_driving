# utils.py

import cv2
import numpy as np

def roi_for_lane(image):
    """이미지의 하단 부분만 사용하여 ROI를 설정하는 함수"""
    return image[246:396, :]

def process_image(image):
    """이미지 전처리를 수행하는 함수"""
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # 그레이스케일 변환
    
    gaussian_kernel_size = 13
    adaptive_thresh_C = -2.2
    adaptive_thresh_block_size = 9
    canny_lower = 33
    canny_upper = 255
    morph_kernel_size = 1

    # 가우시안 블러
    #13×13
    #강한 블러 → 작은 패턴·노이즈까지 많이 사라짐
    #차선이나 경계가 더 부드럽지만, 너무 강하면 얇은 선이 희미해질 수 있음

    #7×7
    # 중간 정도 블러 → 큰 노이즈는 줄이고, 얇은 선은 보존
    # 속도 훨씬 빠름, 세부 패턴 유지 가능
    blurred_image = cv2.GaussianBlur(gray_img, (gaussian_kernel_size, gaussian_kernel_size), 0)

    # 적응형 이진화
    adaptive_gaussian = cv2.adaptiveThreshold(blurred_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                              cv2.THRESH_BINARY, adaptive_thresh_block_size, adaptive_thresh_C)
    
    # 캐니 에지 검출
    edged = cv2.Canny(adaptive_gaussian, canny_lower, canny_upper)

    # 형태학적 닫기 연산
    # MORPH_CLOSE = 얇은 경계선 사이의 작은 구멍을 메움, Canny Edge에서 끊긴 선을 연결, 경계선이 조금 두꺼워짐 → 이후 슬라이딩윈도우에서 더 잘 잡힘
    # 단점: 너무 큰 커널을 쓰면 세부 형태가 뭉개짐 
    # 3×3 = 9연산, 1×1 = 1연산 → 연산량은 미미 (CPU 부하 거의 없음) 
    # 1X1 = 사실상 아무 효과 없음 (각 픽셀 자기 자신만 연산)
    kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    closed_image = cv2.morphologyEx(edged, cv2.MORPH_CLOSE, kernel)

    return gray_img, blurred_image, adaptive_gaussian, edged, closed_image
    #return None, None, None, None, closed  # 최종 결과만 반환

def warper(image):
    """원근 변환을 수행하는 함수"""
    y, x = image.shape[0:2]
    #y = 480
    #x = 640

    # left_margin_1 = 0
    # top_margin_1 = 130
    
    # left_margin_2 = 180
    # top_margin_2 = 54

    left_margin_1 = 0
    top_margin_1 = 110
    
    left_margin_2 = 185
    top_margin_2 = 39

    # src_point1 = [left_margin_1, top_margin_1]  # 왼쪽 아래 점
    # src_point2 = [left_margin_2, top_margin_2]  # 왼쪽 위 점
    # # src_point3 = [x - left_margin_2, top_margin_2]  # 오른쪽 위 점
    # # src_point4 = [x - left_margin_1, top_margin_1]  # 오른쪽 아래 점

    # src_point3 = [460, top_margin_2]  # 오른쪽 위 점
    # src_point4 = [635, top_margin_1]  # 오른쪽 아래 점

    ################### one_lane_detection_parameter #############
    # src_point1 = [136, 150]  # 왼쪽 아래 점
    # src_point4 = [575, 150]  # 오른쪽 아래 점
    # src_point3 = [485, 71]  # 오른쪽 위 점
    # src_point2 = [255, 70]  # 왼쪽 위 점

    # src_points = np.float32([src_point1, src_point2, src_point3, src_point4])  # 원본 이미지에서의 점들
    
    # dst_point1 = [x/4 -6, y]  # 변환 이미지에서의 왼쪽 아래 점
    # dst_point2 = [x/4 -6 , 0]  # 변환 이미지에서의 왼쪽 위 점
    # dst_point3 = [x/4 *3 -6, 0]  # 변환 이미지에서의 오른쪽 위 점
    # dst_point4 = [x/4 *3 -6, y]  # 변환 이미지에서의 오른쪽 아래 점
    # dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])  # 변환 이미지에서의 점들

######08/26 one lane ###
    src_point1 = [90, 150]  # 왼쪽 아래 점
    src_point2 = [215, 62]  # 왼쪽 위 점
    src_point3 = [506, 62]  # 오른쪽 위 점
    src_point4 = [600, 150]  # 오른쪽 아래 점

    src_points = np.float32([src_point1, src_point2, src_point3, src_point4])  # 원본 이미지에서의 점들

    dst_point1 = [x/4, y]  # 변환 이미지에서의 왼쪽 아래 점
    dst_point2 = [x/4 , 0]  # 변환 이미지에서의 왼쪽 위 점
    dst_point3 = [x/4 * 3, 0]  # 변환 이미지에서의 오른쪽 위 점
    dst_point4 = [x/4 * 3 , y]  # 변환 이미지에서의 오른쪽 아래 점




     ################## chang's code ############
    # src_point1 = [145, 70]  # 왼쪽 아래 점
    # src_point2 = [209, 40]  # 왼쪽 위 점
    # src_point3 = [536, 40]  # 오른쪽 위 점
    # src_point4 = [608, 70]  #른쪽 아래 점
    
    ###### kuac ##### both_lane_too_much_distortion
    # src_point1 = [140, 125]  # 왼쪽 아래 점
    # src_point2 = [258, 60]  # 왼쪽 위 점
    # src_point3 = [507, 35]  # 오른쪽 위 점
    # src_point4 = [640, 78]  # 오른쪽 아래 점

    # src_points = np.float32([src_point1, src_point2, src_point3, src_point4])  # 원본 이미지에서의 점들

    # dst_point1 = [x/4 - 13, y]  # 변환 이미지에서의 왼쪽 아래 점
    # dst_point2 = [x/4 - 13, 0]  # 변환 이미지에서의 왼쪽 위 점
    # dst_point3 = [x/4 * 3 - 13, 0]  # 변환 이미지에서의 오른쪽 위 점
    # dst_point4 = [x/4 *3 - 13, y]  # 변환 이미지에서의 오른쪽 아래 점

    #######chang's ########
    # dst_point1 = [x/4 + 5, y]  # 변환 이미지에서의 왼쪽 아래 점
    # dst_point2 = [x/4 + 5, 0]  # 변환 이미지에서의 왼쪽 위 점
    # dst_point3 = [x/4 *3 -5, 0]  # 변환 이미지에서의 오른쪽 위 점
    # dst_point4 = [x/4 *3 -5, y]  # 변환 이미지에서의 오른쪽 아래 점

    

    # dst_point1 = [0, y]  # 변환 이미지에서의 왼쪽 아래 점
    # dst_point2 = [0, 0]  # 변환 이미지에서의 왼쪽 위 점
    # dst_point3 = [x, 0]  # 변환 이미지에서의 오른쪽 위 점
    # dst_point4 = [x, y]  # 변환 이미지에서의 오른쪽 아래 점

    dst_points = np.float32([dst_point1, dst_point2, dst_point3, dst_point4])  # 변환 이미지에서의 점들

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)  # 원근 변환 행렬 계산
    warped_img = cv2.warpPerspective(image, matrix, (x, y))  # 원근 변환 적용
    
    return warped_img
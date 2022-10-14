
import cv2

file_name = './working_space/data/left_disp_float.bmp'
focal_length = 289.0412621091288
cx = 307.599660807135
cy = 235.6465488645034
base_line = 0.11

left_disp_float = cv2.imread(file_name)
# cv2.imshow("left_disp_float", left_disp_float)

left_disp_backrot = cv2.rotate(left_disp_float, cv2.ROTATE_90_COUNTERCLOCKWISE)
# cv2.imshow("left_disp_backrot", left_disp_backrot)




# cv2.waitKey(0)
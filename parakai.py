# main_rover_control.py 内の detect_red_in_grid 関数

def detect_red_in_grid(picam2_instance, save_path="/home/mark1/Pictures/akairo_grid.jpg", min_red_pixel_ratio_per_cell=0.10):
    """
    カメラ画像を縦2x横3のグリッドに分割し、各セルでの赤色検出を行い、その位置情報を返します。
    Picamera2のTransformで画像が既に回転されていることを前提とします。
    さらに、ここで必要に応じてソフトウェア的に反転を補正します。
    """
    try:
        frame_rgb = picam2_instance.capture_array() # Picamera2設定で既に回転済み
        if frame_rgb is None:
            print("画像キャプチャ失敗: フレームがNoneです。")
            return 'error_in_processing'

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        
        # Picamera2のtransformで90度回転されているはずなので、ここでは回転処理は不要
        processed_frame_bgr = frame_bgr 

        # ★★★ ここにcv2.flipを追加して左右反転を修正 ★★★
        # もし、画像が左右反転している場合 (水平フリップ)
        processed_frame_bgr = cv2.flip(processed_frame_bgr, 1) # 1 は水平フリップ (左右反転)
        # もし、画像が上下反転している場合 (垂直フリップ、通常はあまりない)
        # processed_frame_bgr = cv2.flip(processed_frame_bgr, 0) # 0 は垂直フリップ (上下反転)
        # もし、両方反転している場合
        # processed_frame_bgr = cv2.flip(processed_frame_bgr, -1) # -1 は水平垂直両方 (180度回転と同じ)
        # ★★★ 修正ここまで ★★★
        
        height, width, _ = processed_frame_bgr.shape # 処理後のフレームのサイズを使う
        cell_height = height // 2 ; cell_width = width // 3
        cells = {
            'top_left': (0, cell_height, 0, cell_width), 'top_middle': (0, cell_height, cell_width, 2 * cell_width),
            'top_right': (0, cell_height, 2 * cell_width, width),
            'bottom_left': (cell_height, height, 0, cell_width), 'bottom_middle': (cell_height, height, cell_width, 2 * cell_width),
            'bottom_right': (cell_height, height, 2 * cell_width, width),
        }
        red_counts = {key: 0 for key in cells} ; total_pixels_in_cell = {key: 0 for key in cells}

        lower_red1 = np.array([0, 50, 50]) ; upper_red1 = np.array([30, 255, 255])
        lower_red2 = np.array([150, 50, 50]) ; upper_red2 = np.array([180, 255, 255])

        blurred_full_frame = cv2.GaussianBlur(processed_frame_bgr, (5, 5), 0)
        hsv_full = cv2.cvtColor(blurred_full_frame, cv2.COLOR_BGR2HSV)
        mask_full = cv2.bitwise_or(cv2.inRange(hsv_full, lower_red1, upper_red1),
                                   cv2.inRange(hsv_full, lower_red2, upper_red2))
        red_pixels_full = np.count_nonzero(mask_full) ; total_pixels_full = height * width
        red_percentage_full = red_pixels_full / total_pixels_full if total_pixels_full > 0 else 0.0

        if red_percentage_full >= 0.80:
            print(f"画像全体の赤色ピクセル割合: {red_percentage_full:.2%} (高割合) -> high_percentage_overall")
            cv2.imwrite(save_path, processed_frame_bgr)
            return 'high_percentage_overall'

        debug_frame = processed_frame_bgr.copy()
        for cell_name, (y_start, y_end, x_start, x_end) in cells.items():
            cell_frame = processed_frame_bgr[y_start:y_end, x_start:x_end]
            blurred_cell_frame = cv2.GaussianBlur(cell_frame, (5, 5), 0)
            hsv_cell = cv2.cvtColor(blurred_cell_frame, cv2.COLOR_BGR2HSV)
            mask_cell = cv2.bitwise_or(cv2.inRange(hsv_cell, lower_red1, upper_red1),
                                       cv2.inRange(hsv_cell, lower_red2, upper_red2))
            red_counts[cell_name] = np.count_nonzero(mask_cell)
            total_pixels_in_cell[cell_name] = cell_frame.shape[0] * cell_frame.shape[1]
            
            color = (255, 0, 0) ; thickness = 2
            if red_counts[cell_name] / total_pixels_in_cell[cell_name] >= min_red_pixel_ratio_per_cell:
                color = (0, 0, 255) ; thickness = 3
            cv2.rectangle(debug_frame, (x_start, y_start), (x_end, y_end), color, thickness)
            cv2.putText(debug_frame, f"{cell_name}: {(red_counts[cell_name] / total_pixels_in_cell[cell_name]):.2f}", 
                        (x_start + 5, y_start + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        directory = os.path.dirname(save_path)
        if not os.path.exists(directory): os.makedirs(directory)
        cv2.imwrite(save_path, debug_frame)
        print(f"グリッド検出画像を保存しました: {save_path}")

        bottom_left_ratio = red_counts['bottom_left'] / total_pixels_in_cell['bottom_left']
        bottom_middle_ratio = red_counts['bottom_middle'] / total_pixels_in_cell['bottom_middle']
        bottom_right_ratio = red_counts['bottom_right'] / total_pixels_in_cell['bottom_right']

        detected_cells = []
        if bottom_left_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_left')
        if bottom_middle_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_middle')
        if bottom_right_ratio >= min_red_pixel_ratio_per_cell: detected_cells.append('bottom_right')

        if len(detected_cells) == 0:
            print("赤色を検出しませんでした (下段)")
            return 'none_detected'
        elif 'bottom_left' in detected_cells and 'bottom_right' not in detected_cells:
            print("赤色が左下に偏って検出されました")
            return 'left_bottom'
        elif 'bottom_right' in detected_cells and 'bottom_left' not in detected_cells:
            print("赤色が右下に偏って検出されました")
            return 'right_bottom'
        elif 'bottom_left' in detected_cells and 'bottom_middle' in detected_cells and 'bottom_right' in detected_cells:
            print("赤色が下段全体に広く検出されました")
            return 'bottom_middle'
        elif 'bottom_middle' in detected_cells:
            print("赤色が下段中央に検出されました")
            return 'bottom_middle'
        else:
            print("赤色が下段の特定の場所に検出されましたが、左右の偏りはありません")
            return 'bottom_middle'

    except Exception as e:
        print(f"カメラ撮影・グリッド処理中にエラーが発生しました: {e}")
        return 'error_in_processing'

# ... (turn_to_relative_angle 関数とメインシーケンスは省略 - 変更なし) ...

# 📦 HƯỚNG DẪN CHẠY DỰ ÁN TRÊN MÁY MỚI (CHƯA CÓ GÌ)

Để chạy được con robot này trên một máy tính Windows hoàn toàn mới, bạn của bạn cần làm theo các bước sau. Đừng lo, mình đã làm sẵn script tự động rồi!

---

## BƯỚC 1: Cài đặt môi trường Windows (Làm 1 lần duy nhất)

1. **Mở PowerShell với quyền Admin:**
    * Bấm phím **Start**, gõ `PowerShell`.
    * Chuột phải vào **Windows PowerShell** -> Chọn **Run as Administrator**.

2. **Chạy lệnh cài WSL:**
    * Copy và Paste lệnh sau vào cửa sổ xanh đó rồi bấm Enter:

        ```powershell
        wsl --install
        ```

    * Máy sẽ tự tải Ubuntu về. Sau khi chạy xong, nó sẽ yêu cầu **Khởi động lại máy tính**. Hãy Restart máy nhé!

3. **Thiết lập Ubuntu (Sau khi khởi động lại):**
    * Sau khi máy lên, một cửa sổ đen Ubuntu sẽ tự hiện ra (hoặc bạn mở nó từ menu Start -> Ubuntu).
    * Nó sẽ hỏi tạo **Username** và **Password**. Hãy đặt tên ngắn gọn (ví dụ: `admin`) và nhớ mật khẩu này nhé!

---

## BƯỚC 2: Cài đặt ROS 2 (Tự động) ⚙️

1. Copy toàn bộ thư mục dự án này (`bt6`) vào ổ `D:\` hoặc `C:\` của máy đó.
2. Mở thư mục dự án ra.
3. Tìm file **`install_ros2.sh`**.
4. Copy file này vào trong Ubuntu bằng cách mở cửa sổ Ubuntu lên và gõ:
    *(Giả sử bạn để thư mục bt6 ở ổ D)*

    ```bash
    cp /mnt/d/bt6/install_ros2.sh ~
    chmod +x ~/install_ros2.sh
    ./install_ros2.sh
    ```

    *(Nếu để chỗ khác thì thay đường dẫn `/mnt/d/...` cho đúng nhé)*

5. **Ngồi chơi xơi nước ☕**: Quá trình này sẽ tự động cài hết mọi thứ cần thiết (ROS 2, Gazebo, thư viện...). Mất khoảng 15-20 phút tùy mạng.

---

## BƯỚC 3: Chạy Robot! 🚗

Sau khi Bước 2 xong xuôi, bạn đó chỉ cần làm y hệt bạn:

1. Vào thư mục dự án trên Windows.
2. Bấm **`run_simulation.bat`** -> Đợi Robot hiện lên.
3. Bấm **`control_robot.bat`** -> Lái xe đi chơi!

---
**Lưu ý:**

* Nếu máy đó chưa cài VS Code hay Git cũng không sao, chỉ cần các bước trên là chạy được mô phỏng rồi.
* Yêu cầu máy có kết nối Internet ổn định ở Bước 2.

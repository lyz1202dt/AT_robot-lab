#include "leg_driver/cdc_trans.hpp"

#include <chrono>
#include <iostream>
#include <thread>

CDCTrans::CDCTrans() {
    _disconnected     = true;
    _handling_events  = true;
    _need_reconnected = false;
    interface_num     = 1;
    hotplug_registered = false;
    transfer_active   = false;
    recv_transfer     = nullptr;
    ctx               = nullptr;
    handle            = nullptr;
    libusb_init(&ctx);
}

CDCTrans::~CDCTrans() {
    stop_events();
    close();
    if (ctx) {
        libusb_exit(ctx);
        ctx = nullptr;
    }
}

bool CDCTrans::open(uint16_t vid, uint16_t pid ){
    last_vid             = vid;
    last_pid             = pid;
    interface_num        = 1;
    handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    std::cout << "[CDCTrans] Opening USB CDC device" << std::endl;
    if (!handle)
    {
        std::cerr << "[CDCTrans] Failed to open USB CDC device" << std::endl;
        return false;
    }
    
    if (libusb_kernel_driver_active(handle, interface_num))
    {
        int ret=libusb_detach_kernel_driver(handle, interface_num);
        std::cout << "[CDCTrans] Detached kernel driver from interface " << interface_num << ", ret=" << ret
                  << std::endl;
    }

    int ret=libusb_claim_interface(handle, interface_num); // 获取通道
    std::cout << "[CDCTrans] Claimed interface " << interface_num << ", ret=" << ret << std::endl;


    // 分配异步传输结构体
    recv_transfer = libusb_alloc_transfer(0);
    std::cout << "[CDCTrans] Allocated async transfer at " << static_cast<void*>(recv_transfer)
              << std::endl;
    
    // 填写异步传输结构体参数
    libusb_fill_bulk_transfer(
        recv_transfer, handle, EP_IN, cdc_rx_buffer,
        sizeof(cdc_rx_buffer),
        [](libusb_transfer* transfer) -> void {
            auto self = static_cast<CDCTrans*>(transfer->user_data);
            //RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"USB传输事件完成，状态%d",transfer->status);
            if (!self->_handling_events) {
                self->transfer_active = false;
                return;
            }
            if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
                self->_disconnected = true;
                self->transfer_active = false;
                return;
            }

            if (self->cdc_recv_cb)
                self->cdc_recv_cb(transfer->buffer, transfer->actual_length);

            int rc = libusb_submit_transfer(transfer);
            if (rc != 0) {
                self->_disconnected = true;
                self->transfer_active = false;
            }
        },
        this, 0);

    // 如果平台支持热插拔
    if (libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
        int rc = libusb_hotplug_register_callback(
            ctx,
            static_cast<libusb_hotplug_event>(
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT | LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED),
            LIBUSB_HOTPLUG_NO_FLAGS, vid, pid, LIBUSB_HOTPLUG_MATCH_ANY,
            [](libusb_context* /*ctx*/, libusb_device* /*device*/, libusb_hotplug_event event,
               void* user_data) -> int {
                static_cast<CDCTrans*>(user_data)->on_hotplug(event);
                return 0;
            },
            this, &hotplug_handle);

        if (rc != LIBUSB_SUCCESS) {}
        else {
            hotplug_registered = true;
        }
    }

    // 提交异步接收请求
    if (libusb_submit_transfer(recv_transfer) != 0) {
        std::cerr << "[CDCTrans] Failed to submit receive transfer" << std::endl;
    } else {
        transfer_active = true;
    }

    _disconnected     = false;
    _handling_events  = true;
    _need_reconnected = false;
    return true;
}

void CDCTrans::close() {
    _disconnected = true;
    _need_reconnected = false;

    if (ctx && hotplug_registered) {
        libusb_hotplug_deregister_callback(ctx, hotplug_handle);
        hotplug_registered = false;
    }

    if (recv_transfer) {
        if (transfer_active) {
            libusb_cancel_transfer(recv_transfer);
            while (transfer_active) {
                timeval tv{};
                tv.tv_sec = 0;
                tv.tv_usec = 50000;
                libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
            }
        }
        libusb_free_transfer(recv_transfer);
        recv_transfer = nullptr;
    }

    if (handle) {
        libusb_release_interface(handle, interface_num);
        libusb_close(handle);
        handle = nullptr;
    }
}

void CDCTrans::stop_events() {
    _handling_events = false;
    _need_reconnected = false;
}

int CDCTrans::send(const uint8_t* data, int size, unsigned int time_out) {
    int actual_size = 0;
    if(_disconnected)
        return -2;
    int rc = libusb_bulk_transfer(handle, EP_OUT, (uint8_t*)data, size, &actual_size, time_out);
    if (rc != 0)
    {
        std::cerr << "[CDCTrans] Bulk send failed: requested=" << size
                  << ", actual=" << actual_size << ", ret=" << rc << std::endl;
        return -1;
    }
    else
        return actual_size;
}

void CDCTrans::process_once() {
    if (!_handling_events || !ctx) {
        return;
    }

    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 50000;   // 50ms
    //RCLCPP_INFO(rclcpp::get_logger("cdc_device"),"进行一次事件处理");
    libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
    if (_disconnected) {
        close();
    }
    if (_need_reconnected && _handling_events) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        open(last_vid, last_pid);
    }
}

void CDCTrans::regeiser_recv_cb(std::function<void(const uint8_t* data, int size)> recv_cb) {
    cdc_recv_cb = std::move(recv_cb);
}

void CDCTrans::on_hotplug(libusb_hotplug_event event) {
    if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT) {
        _disconnected = true;
    } else if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
        _need_reconnected = true;
    }
}

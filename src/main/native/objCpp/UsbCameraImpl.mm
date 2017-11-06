#import <Foundation/Foundation.h>
#import <AVFouncation/AVFoundation.h>

#include <vector>
#include <string>
#include <memory>

#include "cscore_cpp.h"
#include "llvm/StringRef.h"

@interface CaptureDelegate : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
{
  std::weak_ptr<UsbCameraImpl> m_device;
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
  didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
  fromConnection:(AVCaptureConnection *)connection;

@end

@implementation CaptureDelegate

- (void)captureOutput:(AVCaptureOutput *)captureOutput
  didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
  fromConnection:(AVCaptureConnection *)connection {
  (void)captureOutput;
  (void)sampleBuffer;
  (void)connection;

  CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);

  CVPixelBufferLockBaseAddress(imageBuffer, 0);
  void* baseAddr = CVPixelBufferGetBaseAddress(imageBuffer);

  size_t width = CVPixelBufferGetWidth(imageBuffer);
  size_t height = CVPixelBufferGetHeight(imageBuffer);
  size_t rowBytes = CVPixelBufferGetBytesPerRow(imageBuffer);
  OSType pixelFormat = CVPixelBufferGetPixelFormatType(imageBuffer);

  if (rowBytes == 0) {
    CVPixelBufferUnlockBaseAddress(imageBuffer, 0);
    return;
  }

  if (m_device.lock()) {
    auto image = m_device->AllocImage(cs::VideoMode::PixelFormat::kBGR, width, height, width * 3 * height);

    auto cvImage = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
  }

  CVPixelBufferUnlockBaseAddress(imageBuffer, 0);

}

@end

namespace cs {

class UsbCameraImplMac {
 public:
  AVCaptureSession* session;
  AVCaptureDevice* device;
  CaptureDelegate* delegate;
};

UsbCameraImpl::UsbCameraImpl(llvm::StringRef name, llvm::StringRef path)
    : SourceImpl{name},
      m_path{path},
      m_active{false} {
}

UsbCameraImpl::~UsbCameraImpl() {
  m_active = false;
  m_macData = nullptr;
}

void UsbCameraImpl::Start() {
  m_cameraThread = std::thread(&UsbCameraImpl::CameraThreadMain, this);
}

void UsbCameraImpl::CameraThreadMain() {

  while (m_active) {
    if (!m_macData) {
      DeviceConnect();
    }
    auto timeoutTime =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(1.0);
    {
      std::unique_lock<std::mutex> dataLock(m_dataMutex);
      m_dataCv.wait_until(dataLock, timeouttime);
    }
    auto now = wpi::Now();
    if (m_macData && m_lastUpdate + 100000 < now) {
      // Reset macData to cause a reconnect
      m_macData = nullptr;
    }
    // TODO: Handle Properties
  }
}

void UsbCameraImpl::DeviceConnect() {
  auto devices = [ AVCaptureDevice devices];
  for (id object in devices) {
    if (true) {// Item matches {
      // Found item, create everything

    }
  }
}


CS_Source CreateUsbCameraDev(llvm::StringRef name, int dev, CS_Status* status) {
  // Find device
  llvm::StringRef path;
  return CreateCameraFromCaptureDevice(nullptr, name, path, status);
}

CS_Source CreateUsbCameraPath(llvm::StringRef name, llvm::StringRef path,
                              CS_Status* status) {
  auto source = std::make_shared<UsbCameraImpl>(name,
      std::make_unique<UsbCameraImplMac>());
  auto handle = Sources::GetInstance().Allocate(CS_SOURCE_USB, source);
  Notifier::GetInstance().NotifySource(name, handle, CS_SOURCE_CREATED);
  source->Start();
  return handle;
}

std::string GetUsbCameraPath(CS_Source source, CS_Status* status) {
  auto data = Sources::GetInstance().Get(source);
  if (!data || data->kind != CS_SOURCE_USB) {
    *status = CS_INVALID_HANDLE;
    return std::string{};
  }
  return static_cast<UsbCameraImpl&>(*data->source).GetPath();
}

std::vector<UsbCameraInfo> EnumerateUsbCameras(CS_Status* status) {
  std::vector<UsbCameraInfo> retval;
  return retval;
}

}

extern "C" {

CS_Source CS_CreateUsbCameraDev(const char* name, int dev, CS_Status* status) {
  return cs::CreateUsbCameraDev(name, dev, status);
}

CS_Source CS_CreateUsbCameraPath(const char* name, const char* path,
                                 CS_Status* status) {
  return cs::CreateUsbCameraPath(name, path, status);
}

char* CS_GetUsbCameraPath(CS_Source source, CS_Status* status) {
  return ConvertToC(cs::GetUsbCameraPath(source, status));
}

CS_UsbCameraInfo* CS_EnumerateUsbCameras(int* count, CS_Status* status) {
  auto cameras = cs::EnumerateUsbCameras(status);
  CS_UsbCameraInfo* out = static_cast<CS_UsbCameraInfo*>(
      std::malloc(cameras.size() * sizeof(CS_UsbCameraInfo)));
  *count = cameras.size();
  for (size_t i = 0; i < cameras.size(); ++i) {
    out[i].dev = cameras[i].dev;
    out[i].path = ConvertToC(cameras[i].path);
    out[i].name = ConvertToC(cameras[i].name);
  }
  return out;
}

void CS_FreeEnumeratedUsbCameras(CS_UsbCameraInfo* cameras, int count) {
  if (!cameras) return;
  for (int i = 0; i < count; ++i) {
    std::free(cameras[i].path);
    std::free(cameras[i].name);
  }
  std::free(cameras);
}

}  // extern "C"

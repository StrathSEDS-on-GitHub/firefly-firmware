use std::path::PathBuf;


#[derive(Debug, Clone)]
pub struct Device {
    pub name: String,
    pub path: PathBuf,
    pub size: u64,
}

pub fn get_devices() -> Result<impl Iterator<Item = Result<Device, anyhow::Error>>, anyhow::Error> {
    #[cfg(target_family = "unix")]
    {
        let devices = std::fs::read_dir("/sys/class/block/")?;

        Ok(devices.map(|device| -> Result<Device, anyhow::Error> {
            let device = device?;
            let name =
                String::from_utf8_lossy(&device.file_name().into_encoded_bytes()).into_owned();
            let path = PathBuf::from("/dev").join(device.file_name());
            let size = std::str::from_utf8(&std::fs::read(device.path().join("size"))?)?
                .trim()
                .parse::<u64>()?
                * 512; // Linux reports size in 512 byte sectors

            Ok(Device { name, path, size })
        }))
    }

    #[cfg(target_os = "windows")]
    {
        use winapi::um::fileapi::OPEN_EXISTING;
        use winapi::um::winnt::FILE_ATTRIBUTE_DEVICE;

        Ok((0..)
            .map(|i| {
                let file_name = format!("\\\\.\\PhysicalDrive{}", i);
                let handle = unsafe {
                    winapi::um::fileapi::CreateFileW(
                        file_name.encode_utf16().collect::<Vec<_>>().as_ptr(),
                        winapi::um::winnt::GENERIC_READ,
                        winapi::um::winnt::FILE_SHARE_READ | winapi::um::winnt::FILE_SHARE_WRITE,
                        std::ptr::null_mut(),
                        OPEN_EXISTING,
                        FILE_ATTRIBUTE_DEVICE,
                        std::ptr::null_mut(),
                    )
                };

                if handle == winapi::um::handleapi::INVALID_HANDLE_VALUE {
                    // println!("Error {}: {}", file_name, std::io::Error::last_os_error());
                    return None;
                }

                let mut size: winapi::um::winioctl::GET_LENGTH_INFORMATION =
                    unsafe { std::mem::zeroed() };
                let mut bytes: u32 = 0;
                let res = unsafe {
                    winapi::um::ioapiset::DeviceIoControl(
                        handle,
                        winapi::um::winioctl::IOCTL_DISK_GET_LENGTH_INFO,
                        std::ptr::null_mut(),
                        0,
                        &mut size as *mut _ as *mut _,
                        std::mem::size_of_val(&size) as u32,
                        &mut bytes,
                        std::ptr::null_mut(),
                    )
                };

                if res == 0 {
                    // println!("Error IOCTL: {}", std::io::Error::last_os_error());
                    return None;
                }

                Some(Device {
                    name: format!("PhysicalDrive{}", i),
                    path: PathBuf::from(format!("\\\\.\\PhysicalDrive{}", i)),
                    size: *unsafe { size.Length.QuadPart() } as u64,
                })
            })
            .take_while(|x| x.is_some())
            .map(|x| x.ok_or_else(|| unreachable!())))
    }
}
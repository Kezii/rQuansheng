pub enum RadioBound {
    WriteRegister(u16, u16),
    ReadRegister(u16),
}

pub enum HostBound {
    Register(u16, u16),
}

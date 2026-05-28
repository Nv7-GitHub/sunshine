#![allow(dead_code, unused_variables, unused_imports)]

pub enum SourceKind {
    Serial,
    Replay,
    Simulation,
}

pub struct Pipeline {
    pub source: Option<SourceKind>,
}

impl Pipeline {
    pub fn new() -> Self {
        Pipeline { source: None }
    }
}

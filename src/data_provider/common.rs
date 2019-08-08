use crate::errors::*;
use crate::proto;
use std::collections::HashMap;
use std::io::Read;

pub trait DataProvider: Send + Sync {
    fn meta_proto(&self) -> Result<proto::Meta>;
    fn data(
        &self,
        node_id: &str,
        node_attributes: &[&str],
    ) -> Result<HashMap<String, Box<dyn Read>>>;
}

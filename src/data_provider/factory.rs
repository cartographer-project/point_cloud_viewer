use crate::data_provider::{DataProvider, OnDiskDataProvider};
use crate::errors::*;
use fnv::FnvHashMap;
use std::path::Path;

pub type DataProviderFactoryResult = Result<Box<dyn DataProvider>>;
pub type DataProviderFactoryFunction = fn(&str) -> DataProviderFactoryResult;

#[derive(Default, Clone)]
pub struct DataProviderFactory {
    data_provider_fn_map: FnvHashMap<String, DataProviderFactoryFunction>,
}

impl DataProviderFactory {
    pub fn new() -> Self {
        Self {
            data_provider_fn_map: FnvHashMap::default(),
        }
    }

    pub fn register(
        mut self,
        prefix: impl Into<String>,
        function: DataProviderFactoryFunction,
    ) -> DataProviderFactory {
        self.data_provider_fn_map.insert(prefix.into(), function);
        self
    }

    pub fn generate_data_provider(
        &self,
        data_provider_argument: impl AsRef<str>,
    ) -> DataProviderFactoryResult {
        let data_provider_argument = data_provider_argument.as_ref();
        for (prefix, data_provider_factory_function) in &self.data_provider_fn_map {
            if !data_provider_argument.starts_with(prefix) {
                continue;
            }
            return data_provider_factory_function(data_provider_argument);
        }

        // If no data provider was generated, create it from disk
        if Path::new(data_provider_argument).exists() {
            Ok(Box::new(OnDiskDataProvider {
                directory: data_provider_argument.into(),
            }))
        } else {
            Err(format!(
                "Directory '{}' for creating an OnDiskDataProvider doesn't exist.",
                data_provider_argument
            )
            .into())
        }
    }
}

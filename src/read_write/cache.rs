use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::read_write::{Encoding, NodeIterator};
use crate::{AttributeDataType, PointsBatch};
use lru::LruCache;
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::{Arc, Mutex};

const NUM_CACHED_NODES: usize = 25;

pub struct CachedData {
    data: Arc<Vec<PointsBatch>>,
    batch_count: usize,
}

impl Iterator for CachedData {
    type Item = PointsBatch;

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.data.len(), Some(self.data.len()))
    }
    fn next(&mut self) -> Option<PointsBatch> {
        if self.batch_count < self.data.len() {
            let batch = self.data[self.batch_count].clone();
            self.batch_count += 1;
            return Some(batch);
        }
        None
    }
}

pub struct Cache<Id> {
    // TODO: Add batch size and attributes to key
    keys_to_data: LruCache<Id, Arc<Vec<PointsBatch>>>,
}

impl<Id> Default for Cache<Id>
where
    Id: Copy + Eq + Hash,
{
    fn default() -> Cache<Id> {
        Cache {
            keys_to_data: LruCache::new(NUM_CACHED_NODES),
        }
    }
}

impl<Id> Cache<Id>
where
    Id: Copy + Eq + Hash,
{
    pub fn cached_data(
        &mut self,
        id: Id,
        attributes: &[&str],
        batch_size: usize,
    ) -> Option<CachedData> {
        self.keys_to_data.get(&id).map(|d| CachedData {
            data: Arc::clone(&d),
            batch_count: 0,
        })
    }

    pub fn store_batches(&mut self, id: Id, batches: Vec<PointsBatch>) {
        self.keys_to_data.put(id, Arc::new(batches));
    }
}

enum CachedNodeIteratorType {
    Node(NodeIterator),
    Cache(CachedData),
}

pub struct CachedNodeIterator<Id> {
    inner: CachedNodeIteratorType,
    cache: Arc<Mutex<Cache<Id>>>,
    id: Id,
    cached_batches: Vec<PointsBatch>,
}

impl<Id> CachedNodeIterator<Id>
where
    Id: ToString + Copy + Eq + Hash,
{
    pub fn from_data_provider(
        data_provider: &dyn DataProvider,
        attribute_data_types: &HashMap<String, AttributeDataType>,
        encoding: Encoding,
        id: Id,
        num_points: usize,
        batch_size: usize,
        cache: Arc<Mutex<Cache<Id>>>,
    ) -> Result<Self> {
        let attributes: Vec<&str> = attribute_data_types.keys().map(String::as_str).collect();
        Ok(CachedNodeIterator {
            inner: match cache
                .lock()
                .unwrap()
                .cached_data(id, &attributes, batch_size)
            {
                Some(cache_data) => CachedNodeIteratorType::Cache(cache_data),
                None => CachedNodeIteratorType::Node(NodeIterator::from_data_provider(
                    data_provider,
                    attribute_data_types,
                    encoding,
                    &id,
                    num_points,
                    batch_size,
                )?),
            },
            cache: Arc::clone(&cache),
            id,
            cached_batches: Vec::new(),
        })
    }
}

impl<Id> Iterator for CachedNodeIterator<Id>
where
    Id: Copy + Eq + Hash,
{
    type Item = PointsBatch;

    fn size_hint(&self) -> (usize, Option<usize>) {
        match &self.inner {
            CachedNodeIteratorType::Cache(iter) => iter.size_hint(),
            CachedNodeIteratorType::Node(iter) => iter.size_hint(),
        }
    }
    fn next(&mut self) -> Option<PointsBatch> {
        match &mut self.inner {
            CachedNodeIteratorType::Cache(iter) => iter.next(),
            CachedNodeIteratorType::Node(iter) => match iter.next() {
                Some(batch) => {
                    self.cached_batches.push(batch.clone());
                    Some(batch)
                }
                None => {
                    self.cache
                        .lock()
                        .unwrap()
                        .store_batches(self.id, self.cached_batches.split_off(0));
                    None
                }
            },
        }
    }
}

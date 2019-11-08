use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::read_write::{Encoding, NodeIterator};
use crate::{AttributeData, AttributeDataType, PointsBatch};
use cgmath::Vector3;
use lru::LruCache;
use num_integer::div_ceil;
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::{Arc, Mutex};

const NUM_CACHED_NODES: usize = 25;

type CachedPosition = Arc<Vec<Vector3<f64>>>;
type CachedAttributeData = Arc<AttributeData>;

pub struct CachedData {
    position: CachedPosition,
    attributes: HashMap<String, CachedAttributeData>,
    batch_size: usize,
    point_count: usize,
}

impl Iterator for CachedData {
    type Item = PointsBatch;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let num_batches = div_ceil(self.position.len(), self.batch_size);
        (num_batches, Some(num_batches))
    }
    fn next(&mut self) -> Option<PointsBatch> {
        if self.point_count < self.position.len() {
            let num_points_to_read =
                std::cmp::min(self.batch_size, self.position.len() - self.point_count);
            let mut position = Vec::new();
            position.extend_from_slice(
                &self.position[self.point_count..self.point_count + num_points_to_read],
            );
            let attributes = self
                .attributes
                .iter()
                .map(|(a_name, a_data)| {
                    let mut data = AttributeData::from(a_data.data_type());
                    data.extend_from_slice(
                        &a_data,
                        self.point_count,
                        self.point_count + num_points_to_read,
                    )
                    .unwrap();
                    (a_name.clone(), data)
                })
                .collect();
            self.point_count += num_points_to_read;
            return Some(PointsBatch {
                position,
                attributes,
            });
        }
        None
    }
}

pub struct Cache<Id> {
    ids_to_pos: LruCache<Id, CachedPosition>,
    ids_to_data: LruCache<(Id, String), CachedAttributeData>,
}

impl<Id> Default for Cache<Id>
where
    Id: Copy + Eq + Hash,
{
    fn default() -> Cache<Id> {
        Cache {
            ids_to_pos: LruCache::new(NUM_CACHED_NODES),
            ids_to_data: LruCache::new(NUM_CACHED_NODES),
        }
    }
}

impl<Id> Cache<Id>
where
    Id: Copy + Eq + Hash,
{
    fn position(&mut self, id: Id) -> Option<CachedPosition> {
        self.ids_to_pos
            .get(&id)
            .map(|pos_data| Arc::clone(&pos_data))
    }

    fn attribute_data(
        &mut self,
        id: Id,
        attributes: &[&str],
    ) -> HashMap<String, CachedAttributeData> {
        attributes
            .iter()
            .filter_map(|attrib| {
                self.ids_to_data
                    .get(&(id, attrib.to_string()))
                    .map(|attr_data| (attrib.to_string(), Arc::clone(&attr_data)))
            })
            .collect()
    }

    pub fn cached_data(
        &mut self,
        id: Id,
        attributes: &[&str],
        batch_size: usize,
    ) -> Option<CachedData> {
        self.position(id).and_then(|position| {
            let cached_attribs = self.attribute_data(id, attributes);
            let has_all_attributes = attributes
                .iter()
                .all(|attrib| cached_attribs.contains_key(*attrib));
            if has_all_attributes {
                Some(CachedData {
                    position,
                    attributes: cached_attribs,
                    batch_size,
                    point_count: 0,
                })
            } else {
                None
            }
        })
    }

    pub fn store_batch(&mut self, id: Id, batch: PointsBatch) {
        self.ids_to_pos.put(id, Arc::new(batch.position));
        for (a_name, a_data) in batch.attributes {
            self.ids_to_data.put((id, a_name), Arc::new(a_data));
        }
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
    cached_batch: PointsBatch,
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
            cached_batch: PointsBatch::default(),
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
                    self.cached_batch
                        .extend_from_slice(&batch, 0, batch.position.len())
                        .unwrap();
                    Some(batch)
                }
                None => {
                    self.cache
                        .lock()
                        .unwrap()
                        .store_batch(self.id, self.cached_batch.split_off(0));
                    None
                }
            },
        }
    }
}

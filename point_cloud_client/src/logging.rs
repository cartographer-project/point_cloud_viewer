use lazy_static::lazy_static;
use point_viewer::errors::*;
use point_viewer::iterator::PointQuery;
use rand::{self, Rng};
use serde_json::{self, Value};
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::sync::Mutex;

const QUERY_LOGFILE_VAR: &str = "POINT_CLOUD_CLIENT_QUERY_LOGFILE";
const QUERY_LOG_SIZE: usize = 1000;

struct PointQueryLog {
    log: Vec<Value>,
    logfile: PathBuf,
    i: usize,
}

impl PointQueryLog {
    fn new() -> Option<Self> {
        let logfile: PathBuf = std::env::var(QUERY_LOGFILE_VAR).ok()?.into();
        if logfile.parent()?.is_dir() {
            let log: Vec<Value> = Vec::with_capacity(QUERY_LOG_SIZE);
            Some(Self { log, logfile, i: 0 })
        } else {
            None
        }
    }

    fn add<'a>(&mut self, query: &PointQuery<'a>) -> Result<()> {
        let value =
            serde_json::to_value(query).map_err(|_| "Could not serialize logged PointQuery.")?;
        if self.log.len() == QUERY_LOG_SIZE {
            // Reservoir sampling
            let j = rand::thread_rng().gen_range(0, self.i + 1);
            if j < QUERY_LOG_SIZE {
                self.log[j] = value;
            }
        } else {
            self.log.push(value);
        }
        self.i += 1;
        Ok(())
    }

    fn flush(&self) -> Result<()> {
        // This truncates the file if it existed
        let file = File::create(&self.logfile)?;
        let writer = BufWriter::new(file);
        serde_json::to_writer(writer, &self.log)
            .map_err(|_| "Could not write point queries to logfile.")?;
        Ok(())
    }
}

// If logging the queries is activated, accumulate queries in memory until a certain number is reached.
//
// We could have this live inside the `PointCloudClient` – doing so wouldn't make the functions
// require `&mut self` and so sharing a `PointCloudClient` between threads is still possible. But
// then if there are multiple instances of `PointCloudClient`, they might race when writing to the
// file named by `POINT_CLOUD_CLIENT_QUERY_LOGFILE`. Therefore, use a global mutex and use that to
// synchronize writing to the file.
lazy_static! {
    static ref QUERY_LOG: Option<Mutex<PointQueryLog>> = { PointQueryLog::new().map(Mutex::new) };
}

pub fn log_if_env_var_is_set<'a>(query: &PointQuery<'a>) {
    if let Some(pql_mutex) = QUERY_LOG.as_ref() {
        let mut pql = pql_mutex.lock().unwrap();
        let _ = pql.add(query);
    }
}

pub fn flush() {
    if let Some(pql_mutex) = QUERY_LOG.as_ref() {
        let pql = pql_mutex.lock().unwrap();
        // If creating the file somehow failed, there's not much we can do
        let _ = pql.flush();
    }
}

use pbr::ProgressBar;
use std::error::Error;
use std::io::Stdout;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const PROGRESS_REFRESH_RATE: Duration = Duration::from_secs(2);

pub fn parse_key_val<T, U>(s: &str) -> Result<(T, U), Box<dyn Error>>
where
    T: FromStr,
    T::Err: Error + 'static,
    U: FromStr,
    U::Err: Error + 'static,
{
    let pos = s
        .find('=')
        .ok_or_else(|| format!("invalid KEY=value: no `=` found in `{}`", s))?;
    Ok((s[..pos].parse()?, s[pos + 1..].parse()?))
}

pub fn create_progress_bar(total: usize, message: &str) -> ProgressBar<Stdout> {
    let mut progress_bar = ProgressBar::new(total as u64);
    progress_bar.set_max_refresh_rate(Some(PROGRESS_REFRESH_RATE));
    progress_bar.message(&format!("{}: ", message));
    progress_bar
}

pub fn create_syncable_progress_bar(
    total: usize,
    message: &str,
) -> Arc<Mutex<ProgressBar<Stdout>>> {
    Arc::new(Mutex::new(create_progress_bar(total, message)))
}

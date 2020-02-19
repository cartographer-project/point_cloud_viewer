use pbr::ProgressBar;
use std::error::Error;
use std::io::Stdout;
use std::str::FromStr;
use std::time::Duration;

pub const PROGRESS_REFRESH_RATE_SECS: u64 = 2;

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
    progress_bar.set_max_refresh_rate(Some(Duration::from_secs(PROGRESS_REFRESH_RATE_SECS)));
    progress_bar.message(&format!("{}: ", message));
    progress_bar
}

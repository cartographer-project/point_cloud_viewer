use crate::read_write::{PlyIterator, PtsIterator};
use crate::Point;
use pbr::ProgressBar;
use std::io::Stdout;
use std::path::PathBuf;

#[derive(Debug)]
pub enum InputFile {
    Ply(PathBuf),
    Pts(PathBuf),
}

pub enum InputFileIterator {
    Ply(PlyIterator),
    Pts(PtsIterator),
}

impl Iterator for InputFileIterator {
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        match *self {
            InputFileIterator::Ply(ref p) => p.size_hint(),
            InputFileIterator::Pts(ref p) => p.size_hint(),
        }
    }

    fn next(&mut self) -> Option<Point> {
        match self {
            InputFileIterator::Ply(p) => p.next(),
            InputFileIterator::Pts(p) => p.next(),
        }
    }
}

pub fn make_stream(input: &InputFile) -> (InputFileIterator, Option<ProgressBar<Stdout>>) {
    let stream = match *input {
        InputFile::Ply(ref filename) => {
            InputFileIterator::Ply(PlyIterator::from_file(filename).unwrap())
        }
        InputFile::Pts(ref filename) => InputFileIterator::Pts(PtsIterator::from_file(filename)),
    };

    let progress_bar = match stream.size_hint() {
        (_, Some(size)) => Some(ProgressBar::new(size as u64)),
        (_, None) => None,
    };
    (stream, progress_bar)
}

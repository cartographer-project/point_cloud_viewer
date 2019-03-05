use actix_web::{error::ResponseError, HttpResponse};
use failure::Fail;
use point_viewer;
use std::error::Error;

#[derive(Fail, Debug)]
pub enum PointsViewerError {
    #[fail(display = "Bad Request: {}", _0)]
    BadRequest(String),
    #[fail(display = "Internal Server Error: {}", _0)]
    InternalServerError(String),
    #[fail(display = "NotFound: {}", _0)]
    NotFound(String),
}

impl ResponseError for PointsViewerError {
    fn error_response(&self) -> HttpResponse {
        match *self {
            PointsViewerError::BadRequest(ref message) => HttpResponse::BadRequest().json(message),
            PointsViewerError::InternalServerError { .. } => HttpResponse::InternalServerError()
                .json("Internal server error, please try again later."),
            PointsViewerError::NotFound(ref message) => HttpResponse::NotFound().json(message),
        }
    }
}

impl From<json::Error> for PointsViewerError {
    fn from(err: json::Error) -> PointsViewerError {
        PointsViewerError::InternalServerError(err.description().to_string())
    }
}

impl From<std::str::Utf8Error> for PointsViewerError {
    fn from(err: std::str::Utf8Error) -> PointsViewerError {
        PointsViewerError::InternalServerError(err.description().to_string())
    }
}

impl From<std::num::ParseIntError> for PointsViewerError {
    fn from(err: std::num::ParseIntError) -> PointsViewerError {
        PointsViewerError::InternalServerError(err.description().to_string())
    }
}

impl From<point_viewer::errors::Error> for PointsViewerError {
    fn from(err: point_viewer::errors::Error) -> PointsViewerError {
        PointsViewerError::InternalServerError(err.description().to_string())
    }
}
impl From<std::path::StripPrefixError> for PointsViewerError {
    fn from(err: std::path::StripPrefixError) -> PointsViewerError {
        PointsViewerError::InternalServerError(err.description().to_string())
    }
}

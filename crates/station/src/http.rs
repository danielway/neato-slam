//! A deliberately tiny HTTP/1.1 server, enough for one page, a stream of
//! frames and a control endpoint, so the station needs no web framework.
//!
//! - `GET /` serves the page.
//! - `GET /events` streams frames as server-sent events.
//! - `POST /control` takes one JSON [`Control`].
//! - `GET /recordings` lists the logs that can be replayed, as JSON.
//! - `GET /floor.mjpg` streams the ceiling camera's top-down view of the
//!   floor as it changes, when the station has a camera.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::Result;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{mpsc, watch};

use crate::recorder;
use crate::session::Control;

const INDEX_HTML: &str = include_str!("../web/index.html");
/// Longest request head or body accepted, in bytes.
const MAX_REQUEST: usize = 16 * 1024;

pub async fn serve(
    listener: TcpListener,
    controls: mpsc::Sender<Control>,
    frames: watch::Receiver<Arc<str>>,
    recordings: PathBuf,
    floor: Option<PathBuf>,
) -> Result<()> {
    let recordings: Arc<Path> = recordings.into();
    let floor: Option<Arc<Path>> = floor.map(Into::into);
    loop {
        let (stream, _) = listener.accept().await?;
        let (controls, frames, recordings) = (controls.clone(), frames.clone(), recordings.clone());
        let floor = floor.clone();
        tokio::spawn(async move {
            // A browser dropping a connection is routine, not worth a log line.
            let _ = handle(stream, controls, frames, &recordings, floor.as_deref()).await;
        });
    }
}

struct Request {
    method: String,
    path: String,
    json: bool,
    body: Vec<u8>,
}

async fn read_request(reader: &mut BufReader<TcpStream>) -> std::io::Result<Option<Request>> {
    let mut line = String::new();
    if reader.read_line(&mut line).await? == 0 {
        return Ok(None);
    }
    let mut parts = line.split_whitespace();
    let method = parts.next().unwrap_or_default().to_owned();
    let path = parts.next().unwrap_or_default().to_owned();

    let mut head_len = line.len();
    let (mut content_length, mut json) = (0, false);
    loop {
        line.clear();
        head_len += reader.read_line(&mut line).await?;
        if head_len > MAX_REQUEST {
            return Ok(None);
        }
        let header = line.trim_end();
        if header.is_empty() {
            break;
        }
        let Some((name, value)) = header.split_once(':') else {
            continue;
        };
        let value = value.trim();
        if name.eq_ignore_ascii_case("content-length") {
            content_length = value.parse().unwrap_or(usize::MAX);
        } else if name.eq_ignore_ascii_case("content-type") {
            json = value.starts_with("application/json");
        }
    }
    if content_length > MAX_REQUEST {
        return Ok(None);
    }
    let mut body = vec![0; content_length];
    reader.read_exact(&mut body).await?;
    Ok(Some(Request {
        method,
        path,
        json,
        body,
    }))
}

async fn respond(
    stream: &mut TcpStream,
    status: &str,
    content_type: &str,
    body: &[u8],
) -> std::io::Result<()> {
    let head = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {content_type}\r\nContent-Length: {}\r\n\
         Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    );
    stream.write_all(head.as_bytes()).await?;
    stream.write_all(body).await?;
    stream.shutdown().await
}

async fn handle(
    stream: TcpStream,
    controls: mpsc::Sender<Control>,
    mut frames: watch::Receiver<Arc<str>>,
    recordings: &Path,
    floor: Option<&Path>,
) -> std::io::Result<()> {
    let mut reader = BufReader::new(stream);
    let Some(req) = read_request(&mut reader).await? else {
        return Ok(());
    };
    let mut stream = reader.into_inner();
    const TEXT: &str = "text/plain; charset=utf-8";

    match (req.method.as_str(), req.path.as_str()) {
        ("GET", "/") => {
            respond(
                &mut stream,
                "200 OK",
                "text/html; charset=utf-8",
                INDEX_HTML.as_bytes(),
            )
            .await
        }
        ("GET", "/recordings") => {
            let list = serde_json::to_vec(&recorder::list(recordings))?;
            respond(&mut stream, "200 OK", "application/json", &list).await
        }
        ("GET", path) if path.starts_with("/floor.mjpg") => match floor {
            Some(floor) => stream_floor(&mut stream, floor).await,
            None => respond(&mut stream, "404 Not Found", TEXT, b"no camera").await,
        },
        ("GET", "/events") => {
            stream
                .write_all(
                    b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n\
                      Cache-Control: no-store\r\nConnection: keep-alive\r\n\r\n",
                )
                .await?;
            loop {
                let frame = frames.borrow_and_update().clone();
                stream
                    .write_all(format!("data: {frame}\n\n").as_bytes())
                    .await?;
                if frames.changed().await.is_err() {
                    return Ok(());
                }
            }
        }
        // Requiring a JSON content type makes browsers preflight cross-site
        // requests, which this server never approves, so other web pages
        // can't post controls to it.
        ("POST", "/control") if !req.json => {
            respond(
                &mut stream,
                "415 Unsupported Media Type",
                TEXT,
                b"expected application/json",
            )
            .await
        }
        ("POST", "/control") => match serde_json::from_slice::<Control>(&req.body) {
            Ok(control) => {
                let _ = controls.send(control).await;
                respond(&mut stream, "204 No Content", TEXT, b"").await
            }
            Err(e) => {
                respond(
                    &mut stream,
                    "400 Bad Request",
                    TEXT,
                    e.to_string().as_bytes(),
                )
                .await
            }
        },
        _ => respond(&mut stream, "404 Not Found", TEXT, b"not found").await,
    }
}

/// Streams the camera tracker's top-down image as motion JPEG, sending it
/// again each time the tracker replaces the file.
async fn stream_floor(stream: &mut TcpStream, floor: &Path) -> std::io::Result<()> {
    const BOUNDARY: &str = "floor-frame";
    stream
        .write_all(
            format!(
                "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary={BOUNDARY}\r\n\
                 Cache-Control: no-store\r\nConnection: close\r\n\r\n"
            )
            .as_bytes(),
        )
        .await?;
    let mut sent = None;
    loop {
        let modified = tokio::fs::metadata(floor)
            .await
            .and_then(|m| m.modified())
            .ok();
        if modified.is_some()
            && modified != sent
            && let Ok(jpg) = tokio::fs::read(floor).await
        {
            sent = modified;
            let head = format!(
                "--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                jpg.len()
            );
            stream.write_all(head.as_bytes()).await?;
            stream.write_all(&jpg).await?;
            stream.write_all(b"\r\n").await?;
        }
        tokio::time::sleep(std::time::Duration::from_millis(30)).await;
    }
}

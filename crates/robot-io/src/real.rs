use std::io;

use robot_msgs::wire::{self, MAX_FRAME_LEN};
use robot_msgs::{Command, Msg, Pose2, Stamped};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpStream, ToSocketAddrs};

use crate::RobotIo;

/// The physical robot, through `pi-node` over TCP.
pub struct RealBackend {
    stream: TcpStream,
}

impl RealBackend {
    pub async fn connect(addr: impl ToSocketAddrs) -> io::Result<Self> {
        let stream = TcpStream::connect(addr).await?;
        stream.set_nodelay(true)?;
        Ok(Self { stream })
    }
}

fn invalid(e: impl std::error::Error + Send + Sync + 'static) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, e)
}

impl RobotIo for RealBackend {
    async fn recv(&mut self) -> io::Result<Option<Stamped<Msg>>> {
        let len = match self.stream.read_u32_le().await {
            Ok(len) => len as usize,
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e),
        };
        if len > MAX_FRAME_LEN {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("frame of {len} bytes"),
            ));
        }
        let mut body = vec![0; len];
        self.stream.read_exact(&mut body).await?;
        wire::decode_body(&body).map(Some).map_err(invalid)
    }

    async fn send(&mut self, command: Command) -> io::Result<()> {
        let frame = wire::encode_frame(&command).map_err(invalid)?;
        self.stream.write_all(&frame).await
    }

    fn ground_truth(&self) -> Option<Pose2> {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_msgs::WheelPositions;
    use tokio::net::TcpListener;

    #[tokio::test]
    async fn exchanges_frames_with_a_fake_pi() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        let pi = tokio::spawn(async move {
            let (mut socket, _) = listener.accept().await.unwrap();
            let msg = Stamped {
                t_ns: 1,
                seq: 0,
                msg: Msg::Wheels(WheelPositions {
                    left_mm: 5,
                    right_mm: 6,
                }),
            };
            socket
                .write_all(&wire::encode_frame(&msg).unwrap())
                .await
                .unwrap();
            let len = socket.read_u32_le().await.unwrap() as usize;
            let mut body = vec![0; len];
            socket.read_exact(&mut body).await.unwrap();
            wire::decode_body::<Command>(&body).unwrap()
        });

        let mut robot = RealBackend::connect(addr).await.unwrap();
        let got = robot.recv().await.unwrap().unwrap();
        assert_eq!(
            got.msg,
            Msg::Wheels(WheelPositions {
                left_mm: 5,
                right_mm: 6
            })
        );
        robot.send(Command::Stop).await.unwrap();
        assert_eq!(pi.await.unwrap(), Command::Stop);
        assert!(robot.recv().await.unwrap().is_none());
    }
}

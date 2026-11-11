use robot::log::Entry;

use crate::recorder::Recorder;

/// The recording in progress, if any.
#[derive(Default)]
pub(super) struct Tape {
    pub(super) recorder: Option<Recorder>,
    /// Why the last recording failed, for the page.
    pub(super) error: Option<String>,
}

impl Tape {
    /// Writes an entry if recording. The entry is only built when needed.
    pub(super) fn put(&mut self, entry: impl FnOnce() -> Entry) {
        let Some(rec) = &mut self.recorder else {
            return;
        };
        if let Err(e) = rec.write(&entry()) {
            self.error = Some(format!("recording {} stopped: {e}", rec.name));
            self.recorder = None;
        }
    }

    pub(super) fn stop(&mut self) {
        if let Some(rec) = self.recorder.take() {
            let name = rec.name.clone();
            match rec.finish() {
                Ok(()) => println!("station: saved recording {name}"),
                Err(e) => self.error = Some(format!("saving recording {name} failed: {e}")),
            }
        }
    }
}

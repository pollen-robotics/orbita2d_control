use crate::Result;

pub trait CoherentResult<T> {
    fn coherent(self) -> Result<T>;
}

#[derive(Debug)]
struct IncoherentError;
impl std::fmt::Display for IncoherentError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "(incoherent values)",)
    }
}
impl std::error::Error for IncoherentError {}

impl<T> CoherentResult<T> for Result<[T; 2]>
where
    T: Copy + Clone + PartialEq + std::fmt::Debug,
{
    fn coherent(self) -> Result<T> {
        match self {
            Ok([a, b]) if a == b => Ok(a),
            Ok(_) => Err(Box::new(IncoherentError)),
            Err(e) => Err(e),
        }
    }
}

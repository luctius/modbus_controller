use core::cmp::Ordering;
use core::convert::TryInto;
use core::ops;

use derive_more::{Add, AddAssign, Sub, SubAssign, Mul, Sum, Rem, Div, From, Into};

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Add, AddAssign, Sub, SubAssign, Mul, Sum, Div, From, Into)]
pub struct Instant(pub u64);
impl Instant {
    /// Returns the amount of time elapsed from another instant to this one.
    #[must_use]
    pub fn duration_since(self, earlier: Self) -> Duration {
        #[allow(clippy::cast_possible_wrap)]
        let diff: i64 = (self.0 - earlier.0) as i64;
        assert!(diff >= 0, "second instant is later than self");

        if let Ok(duration) = diff.try_into() {
            Duration(duration)
        }
        else {
            Duration(0)
        }
    }
}

impl core::fmt::Display for Instant {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl core::ops::AddAssign<Duration> for Instant {
    fn add_assign(&mut self, rhs: Duration) {
        let ms: MicroSeconds = rhs.into();
        *self += ms
    }
}

impl core::ops::AddAssign<MicroSeconds> for Instant {
    fn add_assign(&mut self, rhs: MicroSeconds) {
        self.0 = self.0.wrapping_add(u64::from(rhs.0) );
    }
}

impl core::ops::Add<Duration> for Instant {
    type Output = Self;

    fn add(mut self, dur: Duration) -> Self {
        self += dur;
        self
    }
}

impl core::ops::SubAssign<Duration> for Instant {
    fn sub_assign(&mut self, dur: Duration) {
        self.0 = self.0.wrapping_sub(u64::from(dur.0) );
    }
}

impl core::ops::Sub<MicroSeconds> for Instant {
    type Output = Self;

    fn sub(self, dur: MicroSeconds) -> Self {
        Self( self.0 - u64::from(dur.0) )
    }
}

impl core::ops::Sub<Duration> for Instant {
    type Output = Self;

    fn sub(self, dur: Duration) -> Self {
        Self(self.0.saturating_sub(u64::from(dur.0) ) )
    }
}

impl Ord for Instant {
    fn cmp(&self, rhs: &Self) -> Ordering {
        self.0.wrapping_sub(rhs.0).cmp(&0)
    }
}

impl PartialOrd for Instant {
    fn partial_cmp(&self, rhs: &Self) -> Option<Ordering> {
        Some(self.cmp(rhs))
    }
}

/// A `Duration` type to represent a span of time in miliseconds.
#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd, Add, AddAssign, Sub, SubAssign, Mul, Sum, Rem, Div, From, Into)]
pub struct Duration(pub u32);

impl From<MicroSeconds> for Duration {
    fn from(us: MicroSeconds) -> Self {
        Self(us.0)
    }
}

impl From<MiliSeconds> for Duration {
    fn from(ms: MiliSeconds) -> Self {
        let us: MicroSeconds = ms.into();
        Self(us.0)
    }
}


impl ops::Add<MicroSeconds> for Duration {
    type Output = Self;

    fn add(self, rhs: MicroSeconds) -> Self {
        Self(self.0 + rhs.0)
    }
}


impl ops::Sub<MicroSeconds> for Duration {
    type Output = Self;

    fn sub(self, rhs: MicroSeconds) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl ops::Rem<Duration> for u32 {
    type Output = Self;

    fn rem(self, rhs: Duration) -> Self {
        self % rhs.0
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd, Add, AddAssign, Sub, SubAssign, Mul, Sum, Rem, Div, From, Into)]
pub struct MicroSeconds(pub u32);
impl MicroSeconds {
    #[must_use]
    pub const fn one() -> Self {
        Self(1)
    }
}

impl From<Duration> for MicroSeconds {
    fn from(f: Duration) -> Self {
        Self(f.0)
    }
}

impl From<MiliSeconds> for MicroSeconds {
    fn from(f: MiliSeconds) -> Self {
        Self(f.0 * 1000)
    }
}

impl core::ops::Add<Duration> for MicroSeconds {
    type Output = Self;

    fn add(self, rhs: Duration) -> Self {
        let us: Self = rhs.into();
        self + us
    }
}

impl core::ops::Div<MicroSeconds> for MicroSeconds {
    type Output = Self;

    fn div(self, rhs: Self) -> Self {
        Self(self.0 / rhs.0)
    }
}

impl core::ops::Rem<MicroSeconds> for MicroSeconds {
    type Output = Self;

    fn rem(self, rhs: Self) -> Self {
        Self(self.0 % rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd, Add, AddAssign, Sub, SubAssign, Mul, Sum, Rem, Div, From, Into)]
pub struct MiliSeconds(pub u32);
impl MiliSeconds {
    #[must_use]
    pub const fn one() -> Self {
        Self(1)
    }
}

impl From<MicroSeconds> for MiliSeconds {
    fn from(f: MicroSeconds) -> Self {
        Self(f.0 / 1000)
    }
}

impl From<Duration> for MiliSeconds {
    fn from(f: Duration) -> Self {
        f.into()
    }
}

impl core::ops::Add<MicroSeconds> for MiliSeconds {
    type Output = Self;

    fn add(self, rhs: MicroSeconds) -> Self {
        let ms: Self = rhs.into();
        self + ms
    }
}

impl core::ops::Div<MiliSeconds> for MiliSeconds {
    type Output = Self;

    fn div(self, rhs: Self) -> Self {
        Self(self.0 / rhs.0)
    }
}

impl core::ops::Rem<MiliSeconds> for MiliSeconds {
    type Output = Self;

    fn rem(self, rhs: Self) -> Self {
        Self(self.0 % rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CharacterDelay {
    T1(MicroSeconds),
    T1_5(MicroSeconds),
    T3_5(MicroSeconds),
}
impl CharacterDelay {
    #[must_use]
    pub fn from_baudrate(baudrate: u32) -> Self {
         // Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
        // for inter character time out and 1.75 ms for a frame delay.
        // For baud rates below 19200 the timeing is more critical and has to be calculated.
        // E.g. 9600 baud in a 11 bit packet is 872.7 characters per second
        // In milliseconds this will be 872.7characters per 1000ms. So for 1 character
        // 1000ms/872.7characters is 1.145833ms per character and finaly modbus states an
        // intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
        // 1.5T = 1.145833ms * 1.5 = 1.71875ms. A frame delay is 3.5T.           
        if baudrate > 192_000_u32 {
            let t1 = (750 * 15) / 10;
            Self::T1(MicroSeconds(t1) )
        }
        else {
            let t1 = (15 * (11_u32 * 1_000_000_u32) / baudrate) / 10;
            Self::T1(MicroSeconds(t1) )
        }
    }

    #[must_use]
    pub fn to_t1(self) -> Self {
        match self {
            Self::T1(_us)  => self,
            Self::T1_5(us) => Self::T1(MicroSeconds( (us.0 * 10) / 15) ),
            Self::T3_5(us) => Self::T1(MicroSeconds( (us.0 * 10) / 35) ),
        }
    }

    #[must_use]
    pub fn to_t1_5(self) -> Self {
        if let Self::T1_5(_) = self { self }
        else if let Self::T1(MicroSeconds(us) ) = self.to_t1() {
            Self::T1_5(MicroSeconds( (us * 15) / 10) )
        }
        else { unreachable!() }
    }

    #[must_use]
    pub fn to_t3_5(self) -> Self {
        if let Self::T3_5(_) = self { self }
        else if let Self::T1(MicroSeconds(us) ) = self.to_t1() {
            Self::T3_5(MicroSeconds( (us * 35) / 10) )
        }
        else { unreachable!() }
    }
}

impl From<CharacterDelay> for MicroSeconds {
    fn from(f: CharacterDelay) -> Self {
        match f {
            CharacterDelay::T1(us) |
                CharacterDelay::T1_5(us) |
                CharacterDelay::T3_5(us) => us,
        }
    }
}

impl From<CharacterDelay> for Duration {
    fn from(f: CharacterDelay) -> Self {
        let us: MicroSeconds = f.into();
        us.into()
    }
}

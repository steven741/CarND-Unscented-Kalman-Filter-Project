module UKF
  ( KF
  , kf_x
  , kf_p
  , kf_t

  , makeFilter
  , predict
  , update
  ) where

import Prelude hiding ((<>))

import App (Sensor (..))

import Numeric.LinearAlgebra


data KF = KF
  { kf_x :: Vector Double
  , kf_p :: Matrix Double
  , kf_t :: Word
  } deriving Show


makeFilter (Laser px py t) =
  KF { kf_x = 5 |> [px,
                    py,
                    0,
                    0,
                    0]
     , kf_p = (5><5) [0.15, 0.00, 0.0, 0.0, 0.0,
                      0.00, 0.15, 0.0, 0.0, 0.0,
                      0.00, 0.00, 1.0, 0.0, 0.0,
                      0.00, 0.00, 0.0, 1.0, 0.0,
                      0.00, 0.00, 0.0, 0.0, 1.0]
     , kf_t = t }

makeFilter (Radar rho phi rho' t) =
  KF { kf_x = 5 |> [rho  * cos phi,
                    rho  * sin phi,
                    rho',
                    0,
                    0]
     , kf_p = (5><5) [0.15, 0.00, 0.0, 0.0, 0.0,
                      0.00, 0.15, 0.0, 0.0, 0.0,
                      0.00, 0.00, 1.0, 0.0, 0.0,
                      0.00, 0.00, 0.0, 1.0, 0.0,
                      0.00, 0.00, 0.0, 0.0, 1.0]
     , kf_t = t }


-- Standard Deviation Values
std_a      = 2.0
std_yawdd  = 0.3
std_laspx  = 0.15
std_laspy  = 0.15
std_radr   = 0.3
std_radphi = 0.03
std_radrd  = 0.3


-- Unwind angle
normAngle θ
  | θ > pi  = normAngle (θ - 2*pi)
  | θ < -pi = normAngle (θ + 2*pi)
  | otherwise = θ


-- Calculate A st. AA^T = Matrix
sqrtMat :: Matrix Double -> Matrix Double
sqrtMat = tr . chol . trustSym


predict :: Word -> KF -> KF
predict t kf =
  kf
  where
    -- Time inside the filter.
    filterTime =
      kf_t kf

    -- Delta-time. Essentially a timestep.
    dt =
      (fromIntegral (t - filterTime)) / 1000000.0


update (Laser px py t) kf =
  kf


update (Radar rho phi rho' t) kf =
  kf

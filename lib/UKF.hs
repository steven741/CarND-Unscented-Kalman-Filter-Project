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
  KF { kf_x = 5 |> [rho * cos phi,
                    rho * sin phi,
                    rho',
                    0,
                    0]
     , kf_p = (5><5) [0.15, 0.00, 0.0, 0.0, 0.0,
                      0.00, 0.15, 0.0, 0.0, 0.0,
                      0.00, 0.00, 1.0, 0.0, 0.0,
                      0.00, 0.00, 0.0, 1.0, 0.0,
                      0.00, 0.00, 0.0, 0.0, 1.0]
     , kf_t = t }



-- Standard Deviation Tuning Parameters
std_a_     = 2.0
std_yawdd_ = 0.3

std_laspx_  = 0.15
std_laspy_  = 0.15

std_radr_   = 0.3
std_radphi_ = 0.03
std_radrd_  = 0.3



-- Unwind angle
normAngle θ
  | θ > pi  = normAngle (θ - 2*pi)
  | θ < -pi = normAngle (θ + 2*pi)
  | otherwise = θ



-- Calculate A st. m = AA^T
sqrtMat :: Matrix Double -> Matrix Double
sqrtMat = tr . chol . sym



{- constant turn rate and velocity magnitude (CTRV) motion model.

   x is the state augmented with the noise vector
   dt is a reasonable time step
 -}
process :: Vector Double -> Double -> Vector Double
process x dt =
  (5 |> [px, py, vel, yaw, yaw']) + f + v
  where
    -- State vector components
    px   = x ! 0
    py   = x ! 1
    vel  = x ! 2
    yaw  = x ! 3
    yaw' = x ! 4

    -- Noise vector components
    noiseA  = x ! 5
    noiseY' = x ! 6

    -- State model vector
    f = if abs yaw' > 0.001
        then
          5 |> [vel/yaw' * ((sin (yaw + yaw' * dt)) - (sin yaw)),
                vel/yaw' * ((cos yaw) - (cos (yaw + yaw' * dt))),
                0,
                yaw' * dt,
                0]
        else
          -- The special case where yaw rate is 0.
          -- Meaning that the motion is traveling
          -- along a straight line.
          5 |> [vel * dt * (cos yaw),
                vel * dt * (sin yaw),
                0,
                0,
                0]

    -- Noise model vector
    v = 5 |> [0.5 * noiseA * (dt ** 2) * (cos yaw),
              0.5 * noiseA * (dt ** 2) * (sin yaw),
              noiseA * dt,
              0.5 * noiseY' * (dt ** 2),
              noiseY' * dt]



{- This gives us KF(k+1 | k) for the filter.
 -}
predict :: Word -> KF -> KF
predict t kf =
  KF { kf_x = x'
     , kf_p = p'
     , kf_t = t }
  where
    -- Time inside the filter.
    filterTime =
      kf_t kf

    -- Delta-time. Essentially a timestep.
    dt =
      (fromIntegral (t - filterTime)) / 1000000.0

    -- n is the size of the state space with noise components
    n = 7

    -- l is a tuning parameter that controls the distance of the points.
    l = 3 - n

    -- Generate a list of augmented sigma points --
    sigmaPoints :: [Vector Double]
    sigmaPoints =
      let
        x = kf_x kf
        p = kf_p kf

        xAug = 7 |> [x ! 0,
                     x ! 1,
                     x ! 2,
                     x ! 3,
                     x ! 4,
                     0,
                     0]
        pAug = diagBlock [p, std_a_ ** 2, std_yawdd_ ** 2]

        sqrtP = toColumns $ sqrtMat $ (l+n) `scale` pAug
      in
        xAug : map (xAug +) sqrtP ++ map (xAug -) sqrtP

    -- Run the sigma points thru the motion model --
    predictedPoints =
      x0 : xs
      where
        x0 = process (head sigmaPoints) dt
        xs = map (\pnt -> process pnt dt)
                 (tail sigmaPoints)

    -- x(k+1 | k)
    x' :: Vector Double
    x' = foldr (\xn x -> x + (wn `scale` xn))
               (w0 `scale` x0)
               xs
      where
        x0 = head predictedPoints
        w0 = l / (l+n)
        wn = 1 / (2 * (l+n))
        xs = tail predictedPoints

    -- p(k+1 | k)
    p' :: Matrix Double
    p' = foldr (\xn p ->
                  let
                    xn_d  = xn - x'
                    xn_d' = 5 |> [xn_d ! 0,
                                  xn_d ! 1,
                                  xn_d ! 2,
                                  normAngle $ xn_d ! 3,
                                  xn_d ! 4]
                  in
                    p + (wn `scale` asColumn xn_d' <> asRow xn_d'))
               (w0 `scale` asColumn x0_d' <> asRow x0_d')
               xs
      where
        x0    = head predictedPoints
        w0    = l / (l+n)
        x0_d  = x0 - x'
        x0_d' = 5 |> [x0_d ! 0,
                      x0_d ! 1,
                      x0_d ! 2,
                      normAngle $ x0_d ! 3,
                      x0_d ! 4]

        wn = 1 / (2 * (l+n))
        xs = tail predictedPoints



{- For the case of laser measurments we don't need to
   use an unscented transformation for the measurment
   update. Standard kalman filter equations apply just
   fine here. However, we do need an unscented
   transformation for the model prediction. Because that
   is a non-linear model space. This gives us
   KF(k+1 | k+1) for the filter with a laser measurment.
 -}
update (Laser px py t) kf =
  KF { kf_x = x'
     , kf_p = p'
     , kf_t = t }
  where
    kf' = predict t kf

    -- Measurment transition matricies
    h = (2><5) [1, 0, 0, 0, 0,
                0, 1, 0, 0, 0]
    r = (2><2) [std_laspx_ ** 2, 0.00000000,
                0.00000000,      std_laspy_ ** 2]
    y = 2 |> [px, py] - (h #> kf_x kf')

    -- Kalman Filter Equations
    i  = ident 5
    ht = tr h
    s  = h <> kf_p kf' <> ht + r
    si = inv s
    k  = kf_p kf' <> ht <> si

    -- Estimation based on kalman filter gain kf(k+1 | k+1)
    x' = kf_x kf' + (k #> y)
    p' = (i - k <> h) <> kf_p kf'



{- For the case of radar measurments we do
   need to use an unscented transformation as
   the both the mapping to measurment space is
   non-linear. This gives us  KF(k+1 | k+1) for
   the filter
 -}
update (Radar rho phi rho' t) kf =
  kf -- skip
  where
    kf' = predict t kf

    n = 5
    l = 3 - n

    --Calculates the state space point as a radar space point.
    radarModel x =
      let
        _x  = x ! 0
        _y  = x ! 1
        _v  = x ! 2
        _θ  = x ! 3
        _vx = _v * (cos _θ)
        _vy = _v * (sin _θ)

        ρ  = sqrt ((_x ** 2) + (_y ** 2))
        φ  = atan2 _y _x
        ρ' = (_x*_vx + _y*_vy) / ρ
      in
        3 |> [ρ, φ, ρ']

    -- Generate a list of sigma points --
    sigmaPoints :: [Vector Double]
    sigmaPoints =
      let
        x = kf_x kf
        p = kf_p kf

        sqrtP = toColumns $ sqrtMat $ (l+n) `scale` p
      in
        x : map (x +) sqrtP ++ map (x -) sqrtP

    -- Run the sigma points thru the radar model --
    zs = map radarModel sigmaPoints

    -- z(k+1 | k)
    z' :: Vector Double
    z' = foldr (\zn z -> z + (wn `scale` zn))
               (w0 `scale` z0)
               (tail zs)
      where
        z0 = head zs
        w0 = l / (l+n)
        wn = 1 / (2 * (l+n))

    -- s(k+1 | k)
    s' :: Matrix Double
    s' = s + r
      where
        z0 = head zs
        w0 = l / (l+n)
        wn = 1 / (2 * (l+n))
        z0_d  = z0 - z'

        r = (3><3) [std_radr_ ** 2, 0, 0,
                    0, std_radphi_ ** 2, 0,
                    0, 0, std_radrd_ ** 2]

        s = foldr (\zn s ->
                     let
                       zn_d  = zn - z'
                     in
                       s + (wn `scale` asColumn zn_d <> asRow zn_d))
                  (w0 `scale` asColumn z0_d <> asRow z0_d)
                  (tail zs)

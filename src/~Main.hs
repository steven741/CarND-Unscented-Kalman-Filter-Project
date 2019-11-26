-- Predicted σ-Points
predict dt x = do
  let pnt = toList x
  let _px = pnt !! 0
  let _py = pnt !! 1
  let _v  = pnt !! 2
  let _θ  = pnt !! 3
  let _θ' = pnt !! 4
  let _νa = pnt !! 5
  let _νt = pnt !! 6

  if abs _θ' > 0.001
  then
    let
      px = _px + _v/_θ' * ((sin (_θ + _θ' * dt)) - (sin _θ)) + 0.5 * _νa * (dt ** 2) * (cos _θ)
      py = _py + _v/_θ' * ((cos _θ) - (cos (_θ + _θ' * dt))) + 0.5 * _νa * (dt ** 2) * (sin _θ)
      v  = _v + _νa * dt
      θ  = _θ + _θ' * dt + 0.5 * _νt * (dt ** 2)
      θ' = _θ' + _νt * dt
    in
      vector [px, py, v, θ, θ']
  else
    let
      px = _px + _v * dt * (cos _θ) + 0.5 * _νa * (dt ** 2) * (cos _θ)
      py = _py + _v * dt * (sin _θ) + 0.5 * _νa * (dt ** 2) * (sin _θ)
      v  = _v + _νa * dt
      θ  = _θ + _θ' * dt + 0.5 * _νt * (dt ** 2)
      θ' = _θ' + _νt * dt
    in
      vector [px, py, v, θ, θ']



    {-
     Kalman Filter procedure for laser measurments
     -}
    filter (Laser vals) =
      let
        px   = vals !! 0
        py   = vals !! 1
        curT = vals !! 2
        dt   = ((curT - t)/1000000.0)
      in
        if t == 0
        then do
          -- Initialize Filter
          mainLoop curT (vector [px, py, 0, 0, 0]) p
        else do
          -- Calculate σ-Points
          let _x = vector $ (toList x) ++ [0, 0]
          let _p = diagBlock [p, std_a_ ** 2, std_yawdd_ ** 2]

          let n  = size _x
          let λ  = 3 - n
          let w0 = (fromIntegral λ)/(fromIntegral $ λ+n)
          let w1 = (0.5/(fromIntegral $ λ+n))
          let w  = w0 : replicate (2*n) w1
          let weighted x = zipWith (\x y -> scalar x * y) w x

          let σ1 = asColumn _x + sqrt (fromIntegral $ λ+n) * sqrtMat _p
          let σ2 = asColumn _x - sqrt (fromIntegral $ λ+n) * sqrtMat _p
          let σ  = [_x] ++ toColumns σ1 ++ toColumns σ2

          let σ' = map (predict dt) σ
          let x' = sum $ weighted σ'

          let diff = \x -> do
              let d  = toList $ x - x'
              let x  = d !! 0
              let y  = d !! 1
              let v  = d !! 2
              let θ  = normAngle $ d !! 3
              let θ' = d !! 4
              vector [x, y, v, θ, θ']

          let ds = map diff σ'
          let xd = map asColumn ds
          let xt = map asRow ds
          let p' = sum $ weighted (zipWith (*) xd xt)


          -- Measurment Update
          let measurementSpace = \x -> do
              let pnt = toList x
              let _x  = pnt !! 0
              let _y  = pnt !! 1

              vector [_x, _y]

          let _zσ = map measurementSpace σ'
          let _z  = sum $ weighted _zσ

          let zDiff = \σ -> σ - _z

          let _zDif = map zDiff _zσ
          let _zd   = map asColumn _zDif
          let _zt   = map asRow _zDif
          let _r    = matrix 2 [std_laspx_*std_laspx_, 0, std_laspy_*std_laspy_, 0]
          let _s    = (sum $ weighted (zipWith (*) _zd _zt)) + _r

          -- Update
          let tcDiff = \z x -> do
              let zd = z - _z
              let xd = x - x'

              asColumn xd * asRow zd

          let _tc = sum $ weighted $ zipWith tcDiff _zσ σ'
          let _k = _tc <> (inv _s)
          let _y = asColumn $ vector [px, py] -_z

          let x'' = flatten $ asColumn x' + _k <> _y
          let p'' = p' - _k <> _s <> (tr _k)

          let xOut = toList x'' !! 0
          let yOut = toList x'' !! 1
          writeSim xOut yOut
          mainLoop curT x'' p''




    {-
     Kalman Filter procedure for radar measurments
     -}
    filter (Radar vals) =
      let
        rho  = vals !! 0
        phi  = vals !! 1
        rho' = vals !! 2
        curT = vals !! 3
        dt   = ((curT - t)/1000000.0)
      in
        if t == 0
        then do
          -- Initialize Filter
          let px = rho  * (cos phi)
          let py = rho  * (sin phi)

          mainLoop curT (vector [px, py, rho', 0, 0]) p
        else do
          -- Calculate σ-Points
          let _x = vector $ (toList x) ++ [0, 0]
          let _p = diagBlock [p, std_a_ ** 2, std_yawdd_ ** 2]

          let n  = size _x
          let λ  = 3 - n
          let w0 = (fromIntegral λ)/(fromIntegral $ λ+n)
          let w1 = (0.5/(fromIntegral $ λ+n))
          let w  = w0 : replicate (2*n) w1

          let σ1 = asColumn _x + sqrt (fromIntegral $ λ+n) * sqrtMat _p
          let σ2 = asColumn _x - sqrt (fromIntegral $ λ+n) * sqrtMat _p
          let σ  = [_x] ++ toColumns σ1 ++ toColumns σ2

          let σ' = map (predict dt) σ
          let x' = sum $ zipWith (\x y -> scalar x * y) w σ'

          let diff = \σ -> do
              let d  = toList $ σ - x'
              let x  = d !! 0
              let y  = d !! 1
              let v  = d !! 2
              let θ  = normAngle $ d !! 3
              let θ' = d !! 4
              vector [x, y, v, θ, θ']

          let difs = map diff σ'
          let xd   = map asColumn difs
          let xt   = map asRow difs
          let p'   = sum (zipWith (\x y -> scalar x * y) w (zipWith (*) xd xt))


          -- Measurment Update
          let measurementSpace = \x -> do
              let pnt = toList x
              let _x  = pnt !! 0
              let _y  = pnt !! 1
              let _v  = pnt !! 2
              let _θ  = pnt !! 3
              let _vx = _v * (cos _θ)
              let _vy = _v * (sin _θ)

              let ρ  = sqrt ((_x ** 2) + (_y ** 2))
              let φ  = atan2 _y _x
              let ρ' = (_x*_vx + _y*_vy) / ρ
              vector [ρ, φ, ρ']

          let _zσ = map measurementSpace σ'
          let _z  = sum (zipWith (\x y -> scalar x * y) w _zσ)

          let zDiff = \σ -> do
              let d  = toList $ σ - _z
              let ρ  = d !! 0
              let φ  = normAngle $ d !! 1
              let ρ' = d !! 2
              vector [ρ, φ, ρ']

          let _zDif = map zDiff _zσ
          let _zd   = map asColumn _zDif
          let _zt   = map asRow _zDif
          let _r    = matrix 3 [std_radr_*std_radr_, 0, 0, 0, std_radphi_*std_radphi_, 0, 0, 0, std_radrd_*std_radrd_]
          let _s    = (sum (zipWith (\x y -> scalar x * y) w (zipWith (*) _zd _zt))) + _r

          -- Update
          let tcDiff = \z x -> do
              let d  = toList $ z - _z
              let d' = toList $ x - x'

              let ρ  = d !! 0
              let φ  = normAngle $ d !! 1
              let ρ' = d !! 2

              let x  = d' !! 0
              let y  = d' !! 1
              let v  = d' !! 2
              let θ  = normAngle $ d' !! 3
              let θ' = d' !! 4

              asColumn (vector [x, y, v, θ, θ']) * asRow (vector [ρ, φ, ρ'])

          let _tc = sum $ zipWith (\x y -> scalar x * y) w $ zipWith tcDiff _zσ σ'
          let _k = _tc <> (inv _s)
          let _y = asColumn $ vector [rho, phi, rho'] - _z

          let x'' = flatten $ asColumn x' + _k <> _y
          let p'' = p' - _k <> _s <> (tr _k)

          let xOut = toList x'' !! 0
          let yOut = toList x'' !! 1
          writeSim xOut yOut
          mainLoop curT x'' p''

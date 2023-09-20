package frc.robot.Library.Vision.Limelight;

class LimeLightFiltering {

  // Limelight Filtering
  private double[] limelight_filterConst_1 = {0.1, 0.1, 0.1};
  private double[] limelightTrgtRelPos_1 = {0.0, 0.0, 0.0};
  private double[] limelightTrgtRelPos_prev_1 = {0.0, 0.0, 0.0};

  private double[] limelight_filterConst_2 = {0.1, 0.1, 0.1};
  private double[] limelightTrgtRelPos_2 = {0.0, 0.0, 0.0};
  private double[] limelightTrgtRelPos_prev_2 = {0.0, 0.0, 0.0};

  // getLimeLightTrgtRelPosFiltered
  // 	@trgtID - Limelight Target ID
  //
  // returns the x, y and z filtered relative distances to the target from the Limelight Camera
  // Position

  public void setLimeLight_filterConst(int trgtID, double a1, double a2, double a3) {
    double[] a = {0.0, 0.0, 0.0};

    a[1] = a1;
    a[2] = a2;
    a[3] = a3;

    if (trgtID == 1) {
      this.limelight_filterConst_1 = a;
    } else if (trgtID == 2) {
      this.limelight_filterConst_2 = a;
    }
  }

  public double[] getLimeLight_filterConst(int trgtID) {
    double[] a = {0.0, 0.0, 0.0};

    if (trgtID == 1) {
      a = this.limelight_filterConst_1;
    } else if (trgtID == 2) {
      a = this.limelight_filterConst_2;
    }
    return a;
  }

  public void resetLimeLightFilter(int trgtID, double x, double y, double z) {
    double[] y0 = {0.0, 0.0, 0.0};

    y0[0] = x;
    y0[1] = y;
    y0[2] = z;

    if (trgtID == 1) {
      this.limelightTrgtRelPos_1 = y0;
      this.limelightTrgtRelPos_prev_1 = y0;
    } else if (trgtID == 2) {
      this.limelightTrgtRelPos_2 = y0;
      this.limelightTrgtRelPos_prev_2 = y0;
    }
  }

  public double[] getLimeLightTrgtRelPosFiltered(int trgtID) {
    double[] y = {0.0, 0.0, 0.0};
    if (trgtID == 1) {
      y = this.limelightTrgtRelPos_1;
    } else if (trgtID == 2) {
      y = this.limelightTrgtRelPos_2;
    }
    return y;
  }

  public double[] getLimeLightTrgtPosFiltered(int trgtID) {
    // double[] y_Rel = {0.0, 0.0, 0.0};
    double[] y = {0.0, 0.0, 0.0};
    // if (trgtID == 1) {
    //	y_Rel = this.limelightTrgtRelPos_1;
    // }else if (trgtID ==2){
    //	y_Rel = this.limelightTrgtRelPos_2;
    // }

    // y = LimeLightSubSys.convertLimeLightRelPos2Pos(y_Rel);

    return y;
  }

  public void updateLimeLightFilter(int trgtID) {
    int i;
    double[] y = {0.0, 0.0, 0.0};
    double[] y0 = y;
    double[] x = y;
    double[] a = y;

    if (trgtID == 1) {
      y0 = this.limelightTrgtRelPos_prev_1;
      a = this.limelight_filterConst_1;
    } else if (trgtID == 2) {
      y0 = this.limelightTrgtRelPos_prev_1;
      a = this.limelight_filterConst_1;
    }

    // TODO fix this
    // x = LimeLightSubSys.getLimeLightTrgtRelPos(trgtID);
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;

    for (i = 0; i < 3; i++) {
      y[i] = y0[i] + a[i] * (x[i] - y0[i]);
    }

    if (trgtID == 1) {
      this.limelightTrgtRelPos_1 = y;
      this.limelightTrgtRelPos_prev_1 = x;
    } else if (trgtID == 2) {
      this.limelightTrgtRelPos_2 = y;
      this.limelightTrgtRelPos_prev_2 = x;
    }
  }
}

package frc.robot.util;

/**
 * Polynomial regression without JAMA dependency. Fits a polynomial y = β0 + β1 x + β2 x^2 + ...
 * using least squares.
 */
public class PolynomialRegression implements Comparable<PolynomialRegression> {

  private final String variableName;
  private final double[] beta;
  private final double sse;
  private final int degree;
  private final double sst;

  public PolynomialRegression(double[] x, double[] y, int degree) {
    this(x, y, degree, "n");
  }

  public PolynomialRegression(double[] x, double[] y, int degree, String variableName) {
    if (x.length != y.length) {
      throw new IllegalArgumentException("x and y arrays must have same length");
    }

    this.variableName = variableName;
    int n = x.length;

    // Build Vandermonde matrix A and vector B
    double[][] A = new double[n][degree + 1];
    double[] B = new double[n];

    for (int i = 0; i < n; i++) {
      B[i] = y[i];
      double v = 1.0;
      for (int j = 0; j <= degree; j++) {
        A[i][j] = v;
        v *= x[i];
      }
    }

    // Solve normal equation (A^T A) β = (A^T B)
    double[][] ATA = new double[degree + 1][degree + 1];
    double[] ATB = new double[degree + 1];

    // Compute A^T * A and A^T * B
    for (int i = 0; i < n; i++) {
      for (int j = 0; j <= degree; j++) {
        ATB[j] += A[i][j] * B[i];
        for (int k = 0; k <= degree; k++) {
          ATA[j][k] += A[i][j] * A[i][k];
        }
      }
    }

    // Solve linear system using Gaussian elimination
    beta = gaussianSolve(ATA, ATB);

    // Compute SST and SSE
    double mean = 0;
    for (double value : y) mean += value;
    mean /= n;
    double sstTmp = 0;
    double sseTmp = 0;
    for (int i = 0; i < n; i++) {
      double diff = y[i] - mean;
      sstTmp += diff * diff;
      double residual = predict(x[i]) - y[i];
      sseTmp += residual * residual;
    }
    sst = sstTmp;
    sse = sseTmp;

    this.degree = degree;
  }

  /** Solve linear system using Gaussian elimination */
  private static double[] gaussianSolve(double[][] A, double[] B) {
    int n = B.length;

    for (int p = 0; p < n; p++) {
      // Pivot
      int max = p;
      for (int i = p + 1; i < n; i++) {
        if (Math.abs(A[i][p]) > Math.abs(A[max][p])) max = i;
      }

      double[] temp = A[p];
      A[p] = A[max];
      A[max] = temp;

      double t = B[p];
      B[p] = B[max];
      B[max] = t;

      // Eliminate
      for (int i = p + 1; i < n; i++) {
        double alpha = A[i][p] / A[p][p];
        B[i] -= alpha * B[p];
        for (int j = p; j < n; j++) {
          A[i][j] -= alpha * A[p][j];
        }
      }
    }

    // Back substitution
    double[] x = new double[n];
    for (int i = n - 1; i >= 0; i--) {
      double sum = B[i];
      for (int j = i + 1; j < n; j++) {
        sum -= A[i][j] * x[j];
      }
      x[i] = sum / A[i][i];
    }

    return x;
  }

  public double beta(int j) {
    if (Math.abs(beta[j]) < 1E-4) return 0.0;
    return beta[j];
  }

  public int degree() {
    return degree;
  }

  public double r2() {
    if (sst == 0) return 1.0;
    return 1.0 - (sse / sst);
  }

  public double predict(double x) {
    double y = 0;
    for (int j = degree; j >= 0; j--) {
      y = beta[j] + x * y;
    }
    return y;
  }

  @Override
  public int compareTo(PolynomialRegression that) {
    double eps = 1E-5;
    int maxDegree = Math.max(this.degree, that.degree);
    for (int j = maxDegree; j >= 0; j--) {
      double t1 = j <= this.degree ? this.beta(j) : 0;
      double t2 = j <= that.degree ? that.beta(j) : 0;
      if (Math.abs(t1) < eps) t1 = 0;
      if (Math.abs(t2) < eps) t2 = 0;
      if (t1 < t2) return -1;
      if (t1 > t2) return 1;
    }
    return 0;
  }

  @Override
  public String toString() {
    StringBuilder s = new StringBuilder();
    int j = degree;

    while (j >= 0 && Math.abs(beta(j)) < 1E-5) j--;

    while (j >= 0) {
      if (j == 0) s.append(String.format("%.7f ", beta(j)));
      else if (j == 1) s.append(String.format("%.7f %s + ", beta(j), variableName));
      else s.append(String.format("%.7f %s^%d + ", beta(j), variableName, j));
      j--;
    }

    s.append("(R^2 = ").append(String.format("%.3f", r2())).append(")");
    return s.toString().replace("+ -", "- ");
  }
}

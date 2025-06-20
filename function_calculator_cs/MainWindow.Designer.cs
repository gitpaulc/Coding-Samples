using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace function_calculator_cs
{
  partial class MainWindow
  {
    private IContainer components = null;

    protected override void Dispose(bool disposing)
    {
      if (disposing && (components != null)) { components.Dispose(); }
      base.Dispose(disposing);
    }

    #region UI Code

    private void InitializeComponent()
    {
      components = new Container();
      AutoScaleMode = AutoScaleMode.Font;
      ClientSize = new Size(800, 600);
      Text = "Function Calculator";
    }

    #endregion
  }
}

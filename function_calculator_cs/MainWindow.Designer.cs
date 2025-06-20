using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace function_calculator_cs
{
  partial class MainWindow
  {
    private Button continueBtn;
    private Button endCurrentTest;
    private TextBox console;
    private IContainer components = null;

    protected override void Dispose(bool disposing)
    {
      if (disposing && (components != null)) { components.Dispose(); }
      base.Dispose(disposing);
    }

    #region UI Code

    private void InitializeComponent()
    {
      continueBtn = new Button();
      endCurrentTest = new Button();
      console = new TextBox();
      SuspendLayout();

      continueBtn.Location = new Point(694, 456);
      continueBtn.Name = "continueBtn";
      continueBtn.Size = new Size(94, 29);
      continueBtn.TabIndex = 0;
      continueBtn.Text = "Continue";
      continueBtn.UseVisualStyleBackColor = true;

      endCurrentTest.Location = new Point(550, 456);
      endCurrentTest.Name = "endCurrentTest";
      endCurrentTest.Size = new Size(138, 29);
      endCurrentTest.TabIndex = 1;
      endCurrentTest.Text = "End Current Test";
      endCurrentTest.UseVisualStyleBackColor = true;

      console.Location = new Point(12, 12);
      console.Multiline = true;
      console.Name = "console";
      console.Size = new Size(776, 438);
      console.ReadOnly = true;
      console.TabIndex = 2;

      AutoScaleDimensions = new SizeF(8F, 20F);
      AutoScaleMode = AutoScaleMode.Font;
      ClientSize = new Size(800, 600);
      Controls.Add(console);
      Controls.Add(endCurrentTest);
      Controls.Add(continueBtn);
      Name = "MainWindow";
      Text = "Function Calculator";

      ResumeLayout(false);
      PerformLayout();
    }

    #endregion // UI Code
  }
}

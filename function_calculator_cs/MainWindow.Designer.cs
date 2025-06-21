/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

using System.ComponentModel;
using System.Drawing;
using System.Windows.Forms;

namespace function_calculator_cs
{

  partial class MainWindow
  {
    private Button continueBtn;
    private Button endCurrentTest;
    private Button testBtn;
    private TextBox console;
    private IContainer components = null;
    private class TestingState
    {
      public Boolean testing = false;
      public int whichTest = 0;
      public int testState = 0;
    }
    TestingState testState = new TestingState();

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
      testBtn = new Button();
      SuspendLayout();

      continueBtn.Location = new Point(694, 456);
      continueBtn.Name = "continueBtn";
      continueBtn.Size = new Size(94, 29);
      continueBtn.TabIndex = 0;
      continueBtn.Text = "Continue";
      continueBtn.UseVisualStyleBackColor = true;
      continueBtn.Click += OnContinue;
 
      endCurrentTest.Location = new Point(550, 456);
      endCurrentTest.Name = "endCurrentTest";
      endCurrentTest.Size = new Size(138, 29);
      endCurrentTest.TabIndex = 1;
      endCurrentTest.Text = "End Current Test";
      endCurrentTest.UseVisualStyleBackColor = true;
      endCurrentTest.Click += OnEndCurrentTest;
 
      console.Location = new Point(12, 12);
      console.Multiline = true;
      console.Name = "console";
      console.ReadOnly = true;
      console.Size = new Size(776, 438);
      console.TabIndex = 2;

      testBtn.Location = new Point(12, 456);
      testBtn.Name = "testBtn";
      testBtn.Size = new Size(94, 29);
      testBtn.TabIndex = 3;
      testBtn.Text = "Run Tests";
      testBtn.UseVisualStyleBackColor = true;
      testBtn.Click += OnTestClicked;

      AutoScaleDimensions = new SizeF(8F, 20F);
      AutoScaleMode = AutoScaleMode.Font;
      ClientSize = new Size(800, 600);
      Controls.Add(testBtn);
      Controls.Add(console);
      Controls.Add(endCurrentTest);
      Controls.Add(continueBtn);
      Name = "MainWindow";
      Text = "Function Calculator";

      ResumeLayout(false);
      PerformLayout();

      endTests();
    }

    private void OnContinue(object sender, EventArgs e)
    {
      if (testState.whichTest >= 0) { testState.testState++; }
      runTests();
    }

    private void OnEndCurrentTest(object sender, EventArgs e)
    {
      testState.whichTest = -1;
      testState.testState = 0;
      console.Text = Environment.NewLine + "Done.";
    }

    private void OnTestClicked(object sender, EventArgs e)
    {
      runTests();
    }

    #endregion // UI Code
  }
}

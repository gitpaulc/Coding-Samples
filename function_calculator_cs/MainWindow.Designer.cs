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
    private MaskedTextBox numberInput;
    private Label enterIntegerLbl;
    private Button okBtn;
    private IContainer components = null;
    private class TestingState
    {
      public Boolean testing = false;
      public int whichTest = 0;
      public int testState = 0;
    }
    TestingState testState = new TestingState();

    private class CalculatorState
    {
      public int outBufferHeight = 0;
      public List<mp> numberStack = new List<mp>();
    }
    CalculatorState calc = new CalculatorState();

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
      numberInput = new MaskedTextBox();
      enterIntegerLbl = new Label();
      okBtn = new Button();
      SuspendLayout();

      continueBtn.Location = new Point(694, 559);
      continueBtn.Name = "continueBtn";
      continueBtn.Size = new Size(94, 29);
      continueBtn.TabIndex = 0;
      continueBtn.Text = "Continue";
      continueBtn.UseVisualStyleBackColor = true;
      continueBtn.Click += OnContinue;

      endCurrentTest.Location = new Point(549, 559);
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

      testBtn.Location = new Point(12, 559);
      testBtn.Name = "testBtn";
      testBtn.Size = new Size(94, 29);
      testBtn.TabIndex = 3;
      testBtn.Text = "Run Tests";
      testBtn.UseVisualStyleBackColor = true;
      testBtn.Click += OnTestClicked;

      numberInput.Location = new Point(195, 453);
      numberInput.Name = "maskedTextBox1";
      numberInput.Size = new Size(593, 27);
      numberInput.TabIndex = 4;
      numberInput.KeyUp += HandleKeyUp;

      enterIntegerLbl.AutoSize = true;
      enterIntegerLbl.Location = new Point(12, 456);
      enterIntegerLbl.Name = "enterIntegerLbl";
      enterIntegerLbl.Size = new Size(161, 20);
      enterIntegerLbl.TabIndex = 5;
      enterIntegerLbl.Text = "Enter a whole number: ";
      enterIntegerLbl.Click += OnNumberLabelClick;
 
      okBtn.Location = new Point(694, 484);
      okBtn.Name = "okBtn";
      okBtn.Size = new Size(94, 29);
      okBtn.TabIndex = 6;
      okBtn.Text = "OK";
      okBtn.UseVisualStyleBackColor = true;
      okBtn.Click += OnOK;

      AutoScaleDimensions = new SizeF(8F, 20F);
      AutoScaleMode = AutoScaleMode.Font;
      ClientSize = new Size(800, 600);
      Controls.Add(okBtn);
      Controls.Add(enterIntegerLbl);
      Controls.Add(numberInput);
      Controls.Add(testBtn);
      Controls.Add(console);
      Controls.Add(endCurrentTest);
      Controls.Add(continueBtn);
      Name = "MainWindow";
      Text = "Function Calculator";

      ResumeLayout(false);
      PerformLayout();
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
      endCurrentTest.Visible = false;
    }

    private void OnTestClicked(object sender, EventArgs e)
    {
      runTests();
    }

    private void OnNumberLabelClick(object sender, EventArgs e) { }

    private void OnOK(object sender, EventArgs e)
    {
      if (!(okBtn.Visible)) { return; }
      var trimmed = numberInput.Text.Trim();
      if (trimmed.Length == 0) { return; }
      if (calc.outBufferHeight > 10)
      {
        console.Text = "";
        calc.outBufferHeight = 0;
      }
      mp numberOut = new mp(0);
      var valid = mp.FromString(trimmed, ref numberOut);
      console.Text += Environment.NewLine;
      if (valid)
      {
        console.Text += numberOut.ToString();
        numberInput.Text = "";
        calc.numberStack.Add(numberOut);
      }
      else { console.Text += "Not a valid number."; }
      ++(calc.outBufferHeight);
    }

    private void HandleKeyUp(object sender, EventArgs e)
    {
      var ee = e as KeyEventArgs;
      if (ee == null) { return; }
      if (ee.KeyCode == Keys.Enter) { OnOK(sender, e); }
    }

    #endregion // UI Code
  }
}

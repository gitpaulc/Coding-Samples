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
    private Button cancelBtn;
    private Button undoBtn;
    private Button redoBtn;
    private Button divisionBtn;
    private Button timesBtn;
    private Button minusBtn;
    private Button plusBtn;
    private Button powerBtn;
    private CheckBox checkSqrt;
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
      public Boolean squareRooting = false;
      public List<mp> numberStack = new List<mp>();
      public List<mp> redoStack = new List<mp>();
      public enum Calculating
      {
        Not,
        Plus,
        Minus,
        Times,
        Div,
        Power
      }
      public Calculating calculating = Calculating.Not;
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
      divisionBtn = new Button();
      timesBtn = new Button();
      minusBtn = new Button();
      plusBtn = new Button();
      powerBtn = new Button();
      cancelBtn = new Button();
      undoBtn = new Button();
      redoBtn = new Button();
      checkSqrt = new CheckBox();
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
      numberInput.Name = "numberInput";
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

      okBtn.Location = new Point(694, 486);
      okBtn.Name = "okBtn";
      okBtn.Size = new Size(94, 29);
      okBtn.TabIndex = 6;
      okBtn.Text = "OK";
      okBtn.UseVisualStyleBackColor = true;
      okBtn.Click += OnOK;

      divisionBtn.Location = new Point(614, 486);
      divisionBtn.Name = "divisionBtn";
      divisionBtn.Size = new Size(34, 29);
      divisionBtn.TabIndex = 7;
      divisionBtn.Text = "÷";
      divisionBtn.UseVisualStyleBackColor = true;
      divisionBtn.Click += OnDiv;

      timesBtn.Location = new Point(574, 486);
      timesBtn.Name = "timesBtn";
      timesBtn.Size = new Size(34, 29);
      timesBtn.TabIndex = 8;
      timesBtn.Text = "×";
      timesBtn.UseVisualStyleBackColor = true;
      timesBtn.Click += OnTimes;

      minusBtn.Location = new Point(534, 486);
      minusBtn.Name = "minusBtn";
      minusBtn.Size = new Size(34, 29);
      minusBtn.TabIndex = 9;
      minusBtn.Text = "-";
      minusBtn.UseVisualStyleBackColor = true;
      minusBtn.Click += OnMinus;

      plusBtn.Location = new Point(494, 486);
      plusBtn.Name = "plusBtn";
      plusBtn.Size = new Size(34, 29);
      plusBtn.TabIndex = 10;
      plusBtn.Text = "+";
      plusBtn.UseVisualStyleBackColor = true;
      plusBtn.Click += OnPlus;

      powerBtn.Location = new Point(653, 486);
      powerBtn.Name = "powerBtn";
      powerBtn.Size = new Size(34, 29);
      powerBtn.TabIndex = 11;
      powerBtn.Text = "^";
      powerBtn.UseVisualStyleBackColor = true;
      powerBtn.Click += OnPower;

      cancelBtn.Location = new Point(694, 521);
      cancelBtn.Name = "cancelBtn";
      cancelBtn.Size = new Size(94, 29);
      cancelBtn.TabIndex = 12;
      cancelBtn.Text = "Cancel";
      cancelBtn.UseVisualStyleBackColor = true;
      cancelBtn.Click += OnCancel;

      undoBtn.Location = new Point(594, 521);
      undoBtn.Name = "undoBtn";
      undoBtn.Size = new Size(94, 29);
      undoBtn.TabIndex = 13;
      undoBtn.Text = "Undo";
      undoBtn.UseVisualStyleBackColor = true;
      undoBtn.Click += OnUndo;

      redoBtn.Location = new Point(195, 486);
      redoBtn.Name = "redoBtn";
      redoBtn.Size = new Size(94, 29);
      redoBtn.TabIndex = 14;
      redoBtn.Text = "Redo";
      redoBtn.UseVisualStyleBackColor = true;
      redoBtn.Click += OnRedo;

      checkSqrt.AutoSize = true;
      checkSqrt.Location = new Point(12, 486);
      checkSqrt.Name = "checkSqrt";
      checkSqrt.Size = new Size(101, 24);
      checkSqrt.TabIndex = 15;
      checkSqrt.Text = "Taking square root?";
      checkSqrt.UseVisualStyleBackColor = true;
      checkSqrt.CheckedChanged += OnCheckSqrt;

      AutoScaleDimensions = new SizeF(8F, 20F);
      AutoScaleMode = AutoScaleMode.Font;
      ClientSize = new Size(800, 600);
      Controls.Add(checkSqrt);
      Controls.Add(redoBtn);
      Controls.Add(undoBtn);
      Controls.Add(cancelBtn);
      Controls.Add(powerBtn);
      Controls.Add(plusBtn);
      Controls.Add(minusBtn);
      Controls.Add(timesBtn);
      Controls.Add(divisionBtn);
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
      if (!incrementTest()) { return; }
      testState.whichTest = -1;
      testState.testState = 0;
      console.Text = Environment.NewLine + "Done.";
      endCurrentTest.Visible = false;
    }

    private void OnTestClicked(object sender, EventArgs e)
    {
      runTests();
    }

    private void OnCheckSqrt(object sender, EventArgs e)
    {
      calc.squareRooting = !calc.squareRooting;
    }

    private void OnNumberLabelClick(object sender, EventArgs e) { }

    private void TrimOutputBuffer()
    {
      if (calc.outBufferHeight > 10)
      {
        console.Text = "";
        calc.outBufferHeight = 0;
      }
    }

    private void OnRedo(object sender, EventArgs e)
    {
      if (!(redoBtn.Visible)) { return; }
      if (calc.redoStack.Count == 0) { redoBtn.Visible = false; return; }
      var recent = new mp(calc.redoStack.Last());
      calc.redoStack.RemoveAt(calc.redoStack.Count - 1);
      calc.numberStack.Add(recent);
      undoBtn.Visible = (calc.numberStack.Count > 0);
      redoBtn.Visible = (calc.redoStack.Count > 0);
      ShowCalcPanel(calc.numberStack.Count > 0);
      TrimOutputBuffer();
      console.Text += Environment.NewLine;
      console.Text += "Redo. Most recent number was ";
      console.Text += recent.ToString();
    }

    private void OnUndo(object sender, EventArgs e)
    {
      if (!(undoBtn.Visible)) { return; }
      if (calc.numberStack.Count == 0) { undoBtn.Visible = false; return; }
      var recent = new mp(calc.numberStack.Last());
      calc.numberStack.RemoveAt(calc.numberStack.Count - 1);
      calc.redoStack.Add(recent);
      redoBtn.Visible = (calc.redoStack.Count > 0);
      TrimOutputBuffer();
      console.Text += Environment.NewLine;
      ShowCalcPanel(calc.numberStack.Count > 0);
      if (calc.numberStack.Count == 0)
      {
        numberInput.Focus();
        console.Text = "Cleared calculator.";
        undoBtn.Visible = false;
        calc.calculating = CalculatorState.Calculating.Not;
        return;
      }
      console.Text += "Most recent number was ";
      console.Text += calc.numberStack[calc.numberStack.Count - 1].ToString();
    }

    private void OnCancel(object sender, EventArgs e)
    {
      if (!(cancelBtn.Visible)) { return; }
      cancelBtn.Visible = false;
      if (calc.calculating != CalculatorState.Calculating.Not)
      {
        ResetCalcPanel(false);
        calc.calculating = CalculatorState.Calculating.Not;
        if (calc.numberStack.Count > 0)
        {
          ShowCalcPanel(true);
        }
      }
      undoBtn.Visible = (calc.numberStack.Count > 0);
      redoBtn.Visible = (calc.redoStack.Count > 0);
    }

    private void OnOK(object sender, EventArgs e)
    {
      if (!(okBtn.Visible)) { return; }
      var trimmed = numberInput.Text.Trim();
      if (trimmed.Length == 0) { return; }
      TrimOutputBuffer();
      mp numberOut = new mp(0);
      var valid = mp.FromString(trimmed, ref numberOut);
      ++(calc.outBufferHeight);
      console.Text += Environment.NewLine;
      if (calc.calculating != CalculatorState.Calculating.Not)
      {
        if ((calc.calculating == CalculatorState.Calculating.Div) && (numberOut == (new mp(0))))
        {
          console.Text += "Cannot divide by zero.";
          return;
        }
        if (calc.numberStack.Count <= 0)
        {
          console.Text += "No previous operand.";
          OnCancel(sender, e);
          return;
        }
        if (!valid) { console.Text += "Not a valid number."; return; }
        mp prev = calc.numberStack[calc.numberStack.Count - 1];
        mp result = new mp();
        if (calc.calculating == CalculatorState.Calculating.Plus)
        {
          console.Text += "Adding...";
          console.Text += Environment.NewLine;
          result = prev + numberOut;
          console.Text += prev.ToString() + " + ";
        }
        else if (calc.calculating == CalculatorState.Calculating.Minus)
        {
          console.Text += "Subtracting...";
          console.Text += Environment.NewLine;
          result = prev - numberOut;
          console.Text += prev.ToString() + " - ";
        }
        else if (calc.calculating == CalculatorState.Calculating.Times)
        {
          console.Text += "Multiplying...";
          console.Text += Environment.NewLine;
          result = prev * numberOut;
          console.Text += prev.ToString() + " × ";
        }
        else if (calc.calculating == CalculatorState.Calculating.Div)
        {
          console.Text += "Dividing...";
          console.Text += Environment.NewLine;
          result = prev / numberOut;
          console.Text += prev.ToString() + " ÷ ";
        }
        else if (calc.calculating == CalculatorState.Calculating.Power)
        {
          console.Text += "Exponentiating...";
          console.Text += Environment.NewLine;
          result = prev.powerOf(numberOut);
          console.Text += prev.ToString() + " to the power of ";
        }
        numberInput.Text = "";
        console.Text += numberOut.ToString() + " = ";
        console.Text += result.ToString();
        if (calc.calculating == CalculatorState.Calculating.Div)
        {
          var remainder = prev % numberOut;
          console.Text += " with a remainder of " + remainder.ToString();
        }
        calc.numberStack.Add(result);
        calc.redoStack.Clear();
        redoBtn.Visible = (calc.redoStack.Count > 0);
        cancelBtn.Visible = false;
        ResetCalcPanel(false);
        calc.calculating = CalculatorState.Calculating.Not;
        if (calc.numberStack.Count > 0)
        {
          ShowCalcPanel(true);
        }
        undoBtn.Visible = (calc.numberStack.Count > 0);
        enterIntegerLbl.Text = "Enter a whole number: ";
        return;
      }
      if (valid)
      {
        console.Text += numberOut.ToString();
        numberInput.Text = "";
        calc.numberStack.Add(numberOut);
        calc.redoStack.Clear();
        redoBtn.Visible = (calc.redoStack.Count > 0);
        undoBtn.Visible = (calc.numberStack.Count > 0);
        ShowCalcPanel(true);
      }
      else { console.Text += "Not a valid number."; }
      enterIntegerLbl.Text = "Enter a whole number: ";
    }

    private void HandleKeyUp(object sender, EventArgs e)
    {
      var ee = e as KeyEventArgs;
      if (ee == null) { return; }
      if (ee.KeyCode == Keys.Enter) { OnOK(sender, e); }
    }

    private void OnPlus(object sender, EventArgs e)
    {
      calc.calculating = CalculatorState.Calculating.Plus;
      enterIntegerLbl.Text = "Add what? ";
      HandleCalculating();
    }

    private void OnMinus(object sender, EventArgs e)
    {
      calc.calculating = CalculatorState.Calculating.Minus;
      enterIntegerLbl.Text = "Subtract what? ";
      HandleCalculating();
    }

    private void OnTimes(object sender, EventArgs e)
    {
      calc.calculating = CalculatorState.Calculating.Times;
      enterIntegerLbl.Text = "Multiply by what? ";
      HandleCalculating();
    }

    private void OnDiv(object sender, EventArgs e)
    {
      calc.calculating = CalculatorState.Calculating.Div;
      enterIntegerLbl.Text = "Divide by what? ";
      HandleCalculating();
    }

    private void OnPower(object sender, EventArgs e)
    {
      calc.calculating = CalculatorState.Calculating.Power;
      enterIntegerLbl.Text = "Raise to what power? ";
      HandleCalculating();
    }

    private void HandleCalculating()
    {
      numberInput.Focus();
      EnableCalcPanel(false);
      okBtn.Text = "=";
      cancelBtn.Visible = true;
      undoBtn.Visible = false;
    }

    private void EnableCalcPanel(Boolean show)
    {
      plusBtn.Enabled = show;
      minusBtn.Enabled = show;
      timesBtn.Enabled = show;
      divisionBtn.Enabled = show;
      powerBtn.Enabled = show;
    }

    private void ShowCalcPanel(Boolean show)
    {
      plusBtn.Visible = show;
      minusBtn.Visible = show;
      timesBtn.Visible = show;
      divisionBtn.Visible = show;
      powerBtn.Visible = show;
    }

    #endregion // UI Code
  }
}

///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __rosout_generated__
#define __rosout_generated__

namespace rxtools{ class RosoutListControl; }
namespace rxtools{ class RosoutPanel; }

#include <wx/sizer.h>
#include <wx/gdicmn.h>
#include <wx/listctrl.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/button.h>
#include <wx/tglbtn.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/bmpbuttn.h>
#include <wx/scrolwin.h>
#include <wx/panel.h>
#include <wx/textctrl.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/spinctrl.h>
#include <wx/dialog.h>
#include <wx/richtext/richtextctrl.h>
#include <wx/listbox.h>
#include <wx/choice.h>
#include <wx/checkbox.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

namespace rxtools
{
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class RosoutPanelBase
	///////////////////////////////////////////////////////////////////////////////
	class RosoutPanelBase : public wxPanel 
	{
		private:
		
		protected:
			rxtools::RosoutListControl* table_;
			wxBoxSizer* severity_sizer_;
			wxButton* clear_button_;
			wxToggleButton* pause_button_;
			wxButton* setup_button_;
			wxButton* m_button5;
			wxBoxSizer* filters_pane_sizer_;
			wxScrolledWindow* filters_window_;
			wxBoxSizer* filters_window_sizer_;
			wxBoxSizer* filters_sizer_;
			wxBitmapButton* add_filter_button_;
			
			// Virtual event handlers, overide them in your derived class
			virtual void onSize( wxSizeEvent& event ){ event.Skip(); }
			virtual void onClear( wxCommandEvent& event ){ event.Skip(); }
			virtual void onPause( wxCommandEvent& event ){ event.Skip(); }
			virtual void onSetup( wxCommandEvent& event ){ event.Skip(); }
			virtual void onNewWindow( wxCommandEvent& event ){ event.Skip(); }
			
		
		public:
			RosoutPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 837,550 ), long style = wxCLIP_CHILDREN|wxTAB_TRAVERSAL );
			~RosoutPanelBase();
		
	};
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class RosoutSetupDialogBase
	///////////////////////////////////////////////////////////////////////////////
	class RosoutSetupDialogBase : public wxDialog 
	{
		private:
		
		protected:
			wxTextCtrl* topic_;
			wxButton* topic_browse_button_;
			wxStaticText* m_staticText2;
			wxSpinCtrl* buffer_size_spinner_;
			wxStdDialogButtonSizer* m_sdbSizer1;
			wxButton* m_sdbSizer1OK;
			wxButton* m_sdbSizer1Cancel;
			
			// Virtual event handlers, overide them in your derived class
			virtual void onTopicBrowse( wxCommandEvent& event ){ event.Skip(); }
			virtual void onCancel( wxCommandEvent& event ){ event.Skip(); }
			virtual void onOk( wxCommandEvent& event ){ event.Skip(); }
			
		
		public:
			RosoutSetupDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Rosout Panel Setup"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 331,214 ), long style = wxDEFAULT_DIALOG_STYLE );
			~RosoutSetupDialogBase();
		
	};
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class TextboxDialog
	///////////////////////////////////////////////////////////////////////////////
	class TextboxDialog : public wxDialog 
	{
		private:
		
		protected:
		
		public:
			wxRichTextCtrl* text_;
			TextboxDialog( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 644,362 ), long style = wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER );
			~TextboxDialog();
		
	};
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class LoggerLevelPanelBase
	///////////////////////////////////////////////////////////////////////////////
	class LoggerLevelPanelBase : public wxPanel 
	{
		private:
		
		protected:
			wxListBox* nodes_box_;
			wxButton* nodes_refresh_;
			wxListBox* loggers_box_;
			wxListBox* levels_box_;
			
			// Virtual event handlers, overide them in your derived class
			virtual void onNodeSelected( wxCommandEvent& event ){ event.Skip(); }
			virtual void onNodesRefresh( wxCommandEvent& event ){ event.Skip(); }
			virtual void onLoggerSelected( wxCommandEvent& event ){ event.Skip(); }
			virtual void onLevelSelected( wxCommandEvent& event ){ event.Skip(); }
			
		
		public:
			LoggerLevelPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxTAB_TRAVERSAL );
			~LoggerLevelPanelBase();
		
	};
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class RosoutTextFilterControlBase
	///////////////////////////////////////////////////////////////////////////////
	class RosoutTextFilterControlBase : public wxPanel 
	{
		private:
		
		protected:
			wxTextCtrl* text_;
			wxChoice* include_exclude_;
			wxCheckBox* regex_;
			wxStaticText* m_staticText2;
			wxCheckBox* message_;
			wxCheckBox* node_;
			wxCheckBox* location_;
			wxCheckBox* topics_;
			
			// Virtual event handlers, overide them in your derived class
			virtual void onText( wxCommandEvent& event ){ event.Skip(); }
			virtual void onIncludeExclude( wxCommandEvent& event ){ event.Skip(); }
			virtual void onRegex( wxCommandEvent& event ){ event.Skip(); }
			virtual void onMessage( wxCommandEvent& event ){ event.Skip(); }
			virtual void onNode( wxCommandEvent& event ){ event.Skip(); }
			virtual void onLocation( wxCommandEvent& event ){ event.Skip(); }
			virtual void onTopics( wxCommandEvent& event ){ event.Skip(); }
			
		
		public:
			RosoutTextFilterControlBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 750,42 ), long style = wxTAB_TRAVERSAL );
			~RosoutTextFilterControlBase();
		
	};
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class RosoutSeverityFilterControlBase
	///////////////////////////////////////////////////////////////////////////////
	class RosoutSeverityFilterControlBase : public wxPanel 
	{
		private:
		
		protected:
			wxStaticText* m_staticText21;
			wxCheckBox* fatal_;
			wxCheckBox* error_;
			wxCheckBox* warn_;
			wxCheckBox* info_;
			wxCheckBox* debug_;
			
			// Virtual event handlers, overide them in your derived class
			virtual void onFatal( wxCommandEvent& event ){ event.Skip(); }
			virtual void onError( wxCommandEvent& event ){ event.Skip(); }
			virtual void onWarn( wxCommandEvent& event ){ event.Skip(); }
			virtual void onInfo( wxCommandEvent& event ){ event.Skip(); }
			virtual void onDebug( wxCommandEvent& event ){ event.Skip(); }
			
		
		public:
			RosoutSeverityFilterControlBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 750,42 ), long style = wxTAB_TRAVERSAL );
			~RosoutSeverityFilterControlBase();
		
	};
	
	///////////////////////////////////////////////////////////////////////////////
	/// Class RosoutFrame
	///////////////////////////////////////////////////////////////////////////////
	class RosoutFrame : public wxFrame 
	{
		private:
		
		protected:
		
		public:
			rxtools::RosoutPanel* rosout_panel_;
			RosoutFrame( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 800,600 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
			~RosoutFrame();
		
	};
	
} // namespace rxtools

#endif //__rosout_generated__

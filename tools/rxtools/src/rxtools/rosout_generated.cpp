///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "rosout_list_control.h"
#include "rosout_panel.h"

#include "rosout_generated.h"

///////////////////////////////////////////////////////////////////////////
using namespace rxtools;

RosoutPanelBase::RosoutPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	this->SetMinSize( wxSize( 273,138 ) );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxHORIZONTAL );
	
	bSizer3->Add( bSizer8, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	bSizer3->Add( bSizer4, 1, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer3, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxVERTICAL );
	
	table_ = new rxtools::RosoutListControl( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_HRULES|wxLC_REPORT|wxLC_SINGLE_SEL|wxLC_VIRTUAL|wxLC_VRULES|wxCLIP_CHILDREN );
	bSizer10->Add( table_, 1, wxALL|wxEXPAND, 5 );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxHORIZONTAL );
	
	severity_sizer_ = new wxBoxSizer( wxHORIZONTAL );
	
	bSizer9->Add( severity_sizer_, 1, wxEXPAND, 5 );
	
	clear_button_ = new wxButton( this, wxID_ANY, wxT("Clear"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( clear_button_, 0, wxALL, 5 );
	
	pause_button_ = new wxToggleButton( this, wxID_ANY, wxT("Pause"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( pause_button_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	setup_button_ = new wxButton( this, wxID_ANY, wxT("Setup"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( setup_button_, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	m_button5 = new wxButton( this, wxID_ANY, wxT("New Window..."), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( m_button5, 0, wxALL, 5 );
	
	bSizer10->Add( bSizer9, 0, wxALIGN_RIGHT|wxEXPAND, 5 );
	
	filters_pane_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	filters_window_ = new wxScrolledWindow( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxVSCROLL );
	filters_window_->SetScrollRate( 30, 30 );
	filters_window_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	filters_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	filters_window_sizer_->Add( filters_sizer_, 1, wxEXPAND, 5 );
	
	add_filter_button_ = new wxBitmapButton( filters_window_, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW );
	add_filter_button_->SetToolTip( wxT("Add Filter") );
	
	add_filter_button_->SetToolTip( wxT("Add Filter") );
	
	filters_window_sizer_->Add( add_filter_button_, 0, wxALL|wxALIGN_RIGHT, 0 );
	
	filters_window_->SetSizer( filters_window_sizer_ );
	filters_window_->Layout();
	filters_window_sizer_->Fit( filters_window_ );
	filters_pane_sizer_->Add( filters_window_, 1, wxALL|wxEXPAND, 5 );
	
	bSizer10->Add( filters_pane_sizer_, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer10, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer2 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_SIZE, wxSizeEventHandler( RosoutPanelBase::onSize ) );
	clear_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onClear ), NULL, this );
	pause_button_->Connect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onPause ), NULL, this );
	setup_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onSetup ), NULL, this );
	m_button5->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onNewWindow ), NULL, this );
}

RosoutPanelBase::~RosoutPanelBase()
{
	// Disconnect Events
	this->Disconnect( wxEVT_SIZE, wxSizeEventHandler( RosoutPanelBase::onSize ) );
	clear_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onClear ), NULL, this );
	pause_button_->Disconnect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onPause ), NULL, this );
	setup_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onSetup ), NULL, this );
	m_button5->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onNewWindow ), NULL, this );
}

RosoutSetupDialogBase::RosoutSetupDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( -1,-1 ), wxDefaultSize );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer2;
	sbSizer2 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Rosout Topic") ), wxVERTICAL );
	
	wxBoxSizer* bSizer91;
	bSizer91 = new wxBoxSizer( wxHORIZONTAL );
	
	topic_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer91->Add( topic_, 1, wxALL|wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );
	
	topic_browse_button_ = new wxButton( this, wxID_ANY, wxT(" ... "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	bSizer91->Add( topic_browse_button_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer2->Add( bSizer91, 1, wxEXPAND, 5 );
	
	bSizer8->Add( sbSizer2, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer21;
	sbSizer21 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Messages") ), wxVERTICAL );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Buffer Size"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	bSizer10->Add( m_staticText2, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	buffer_size_spinner_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 1000000, 20000 );
	bSizer10->Add( buffer_size_spinner_, 0, wxALL, 5 );
	
	sbSizer21->Add( bSizer10, 0, wxALIGN_RIGHT, 5 );
	
	bSizer8->Add( sbSizer21, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxHORIZONTAL );
	
	m_sdbSizer1 = new wxStdDialogButtonSizer();
	m_sdbSizer1OK = new wxButton( this, wxID_OK );
	m_sdbSizer1->AddButton( m_sdbSizer1OK );
	m_sdbSizer1Cancel = new wxButton( this, wxID_CANCEL );
	m_sdbSizer1->AddButton( m_sdbSizer1Cancel );
	m_sdbSizer1->Realize();
	bSizer9->Add( m_sdbSizer1, 1, wxEXPAND|wxALL|wxALIGN_BOTTOM, 5 );
	
	bSizer8->Add( bSizer9, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer8 );
	this->Layout();
	
	// Connect Events
	topic_browse_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onTopicBrowse ), NULL, this );
	m_sdbSizer1Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onCancel ), NULL, this );
	m_sdbSizer1OK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onOk ), NULL, this );
}

RosoutSetupDialogBase::~RosoutSetupDialogBase()
{
	// Disconnect Events
	topic_browse_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onTopicBrowse ), NULL, this );
	m_sdbSizer1Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onCancel ), NULL, this );
	m_sdbSizer1OK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onOk ), NULL, this );
}

TextboxDialog::TextboxDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxVERTICAL );
	
	text_ = new wxRichTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_AUTO_URL|wxTE_READONLY|wxVSCROLL|wxHSCROLL|wxNO_BORDER|wxWANTS_CHARS );
	bSizer11->Add( text_, 1, wxEXPAND | wxALL, 5 );
	
	this->SetSizer( bSizer11 );
	this->Layout();
}

TextboxDialog::~TextboxDialog()
{
}

LoggerLevelPanelBase::LoggerLevelPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	wxStaticBoxSizer* sbSizer3;
	sbSizer3 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Nodes") ), wxVERTICAL );
	
	nodes_box_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer3->Add( nodes_box_, 1, wxALL|wxEXPAND, 5 );
	
	nodes_refresh_ = new wxButton( this, wxID_ANY, wxT("Refresh Nodes"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer3->Add( nodes_refresh_, 0, wxALL|wxEXPAND, 5 );
	
	bSizer12->Add( sbSizer3, 1, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer31;
	sbSizer31 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Loggers") ), wxVERTICAL );
	
	loggers_box_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer31->Add( loggers_box_, 1, wxALL|wxEXPAND, 5 );
	
	bSizer12->Add( sbSizer31, 1, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer311;
	sbSizer311 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Levels") ), wxVERTICAL );
	
	levels_box_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer311->Add( levels_box_, 1, wxALL|wxEXPAND, 5 );
	
	bSizer12->Add( sbSizer311, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer12 );
	this->Layout();
	
	// Connect Events
	nodes_box_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onNodeSelected ), NULL, this );
	nodes_refresh_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoggerLevelPanelBase::onNodesRefresh ), NULL, this );
	loggers_box_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLoggerSelected ), NULL, this );
	levels_box_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLevelSelected ), NULL, this );
}

LoggerLevelPanelBase::~LoggerLevelPanelBase()
{
	// Disconnect Events
	nodes_box_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onNodeSelected ), NULL, this );
	nodes_refresh_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoggerLevelPanelBase::onNodesRefresh ), NULL, this );
	loggers_box_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLoggerSelected ), NULL, this );
	levels_box_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLevelSelected ), NULL, this );
}

RosoutTextFilterControlBase::RosoutTextFilterControlBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );
	
	text_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer14->Add( text_, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxString include_exclude_Choices[] = { wxT("Include"), wxT("Exclude") };
	int include_exclude_NChoices = sizeof( include_exclude_Choices ) / sizeof( wxString );
	include_exclude_ = new wxChoice( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, include_exclude_NChoices, include_exclude_Choices, 0 );
	include_exclude_->SetSelection( 0 );
	bSizer14->Add( include_exclude_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	regex_ = new wxCheckBox( this, wxID_ANY, wxT("Regex"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( regex_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("From"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	m_staticText2->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer14->Add( m_staticText2, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	message_ = new wxCheckBox( this, wxID_ANY, wxT("Message"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( message_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	
	node_ = new wxCheckBox( this, wxID_ANY, wxT("Node"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( node_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	
	location_ = new wxCheckBox( this, wxID_ANY, wxT("Location"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( location_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	
	topics_ = new wxCheckBox( this, wxID_ANY, wxT("Topics"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( topics_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 0 );
	
	this->SetSizer( bSizer14 );
	this->Layout();
	
	// Connect Events
	text_->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( RosoutTextFilterControlBase::onText ), NULL, this );
	include_exclude_->Connect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( RosoutTextFilterControlBase::onIncludeExclude ), NULL, this );
	regex_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onRegex ), NULL, this );
	message_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onMessage ), NULL, this );
	node_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onNode ), NULL, this );
	location_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onLocation ), NULL, this );
	topics_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onTopics ), NULL, this );
}

RosoutTextFilterControlBase::~RosoutTextFilterControlBase()
{
	// Disconnect Events
	text_->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( RosoutTextFilterControlBase::onText ), NULL, this );
	include_exclude_->Disconnect( wxEVT_COMMAND_CHOICE_SELECTED, wxCommandEventHandler( RosoutTextFilterControlBase::onIncludeExclude ), NULL, this );
	regex_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onRegex ), NULL, this );
	message_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onMessage ), NULL, this );
	node_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onNode ), NULL, this );
	location_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onLocation ), NULL, this );
	topics_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutTextFilterControlBase::onTopics ), NULL, this );
}

RosoutSeverityFilterControlBase::RosoutSeverityFilterControlBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText21 = new wxStaticText( this, wxID_ANY, wxT("Severity"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText21->Wrap( -1 );
	m_staticText21->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer14->Add( m_staticText21, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	fatal_ = new wxCheckBox( this, wxID_ANY, wxT("Fatal"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( fatal_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	error_ = new wxCheckBox( this, wxID_ANY, wxT("Error"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( error_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	warn_ = new wxCheckBox( this, wxID_ANY, wxT("Warn"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( warn_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	info_ = new wxCheckBox( this, wxID_ANY, wxT("Info"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( info_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	debug_ = new wxCheckBox( this, wxID_ANY, wxT("Debug"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer14->Add( debug_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	this->SetSizer( bSizer14 );
	this->Layout();
	
	// Connect Events
	fatal_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onFatal ), NULL, this );
	error_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onError ), NULL, this );
	warn_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onWarn ), NULL, this );
	info_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onInfo ), NULL, this );
	debug_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onDebug ), NULL, this );
}

RosoutSeverityFilterControlBase::~RosoutSeverityFilterControlBase()
{
	// Disconnect Events
	fatal_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onFatal ), NULL, this );
	error_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onError ), NULL, this );
	warn_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onWarn ), NULL, this );
	info_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onInfo ), NULL, this );
	debug_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutSeverityFilterControlBase::onDebug ), NULL, this );
}

RosoutFrame::RosoutFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer19;
	bSizer19 = new wxBoxSizer( wxVERTICAL );
	
	rosout_panel_ = new rxtools::RosoutPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer19->Add( rosout_panel_, 1, wxEXPAND | wxALL, 5 );
	
	this->SetSizer( bSizer19 );
	this->Layout();
}

RosoutFrame::~RosoutFrame()
{
}
